/*
 *  NI 16550 Transceiver Driver
 *
 *  The National Instruments (NI) 16550 has built-in RS-485 transceiver control
 *  circuitry. This driver provides the transceiver control functionality
 *  for the RS-485 ports and uses the 8250 driver for the UART functionality.
 *
 *  Copyright 2012 National Instruments Corporation
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the Free
 *  Software Foundation; either version 2 of the License, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 */

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>

#include "8250.h"

#define NI16550_PCR_OFFSET 0x0F
#define NI16550_PCR_RS422 0x00
#define NI16550_PCR_ECHO_RS485 0x01
#define NI16550_PCR_DTR_RS485 0x02
#define NI16550_PCR_AUTO_RS485 0x03
#define NI16550_PCR_WIRE_MODE_MASK 0x03
#define NI16550_PCR_TXVR_ENABLE_BIT (1 << 3)
#define NI16550_PCR_RS485_TERMINATION_BIT (1 << 6)

#define NI16550_PMR_OFFSET 0x0E
/*
 * PMR[1:0] - Port Capabilities
 *
 * 0 - Register not implemented/supported
 * 1 - RS-232 capable
 * 2 - RS-485 capable
 * 3 - RS-232/RS-485 dual-mode capable
 *
 */
#define NI16550_PMR_CAP_MASK   0x03
#define NI16550_PMR_NOT_IMPL   0x00
#define NI16550_PMR_CAP_RS232  0x01
#define NI16550_PMR_CAP_RS485  0x02
#define NI16550_PMR_CAP_DUAL   0x03
/*
 * PMR[4] - Interface Mode
 *
 * 0 - RS-232 mode
 * 1 - RS-485 mode
 *
 */
#define NI16550_PMR_MODE_MASK  0x10
#define NI16550_PMR_MODE_RS232 0x00
#define NI16550_PMR_MODE_RS485 0x10

/*
 * CPR - Clock Prescaler Register
 *
 * [7:3] - Integer part
 * [2:0] - Fractional part (1/8 steps)
 */
#define NI16550_CPR_PRESCALE_1x125	0x9

/* flags for ni16550_device_info */
#define NI_CAP_PMR		0x0001

struct ni16550_device_info {
	unsigned int uartclk;
	uint8_t prescaler;
	int port_type;
	unsigned int flags;
};

struct ni16550_data {
	int line;
};

static int ni16550_enable_transceivers(struct uart_port *port)
{
	uint8_t pcr;

	dev_dbg(port->dev, ">ni16550_enable_transceivers\n");

	pcr = port->serial_in(port, NI16550_PCR_OFFSET);
	pcr |= NI16550_PCR_TXVR_ENABLE_BIT;
	dev_dbg(port->dev, "write pcr: 0x%08x\n", pcr);
	port->serial_out(port, NI16550_PCR_OFFSET, pcr);

	dev_dbg(port->dev, "<ni16550_enable_transceivers\n");

	return 0;
}

static int ni16550_disable_transceivers(struct uart_port *port)
{
	uint8_t pcr;

	dev_dbg(port->dev, ">ni16550_disable_transceivers\n");

	pcr = port->serial_in(port, NI16550_PCR_OFFSET);
	pcr &= ~NI16550_PCR_TXVR_ENABLE_BIT;
	dev_dbg(port->dev, "write pcr: 0x%08x\n", pcr);
	port->serial_out(port, NI16550_PCR_OFFSET, pcr);

	dev_dbg(port->dev, "<ni16550_disable_transceivers\n");

	return 0;
}

static int ni16550_config_rs485(struct uart_port *port,
		struct serial_rs485 *rs485)
{
	uint8_t pcr;

	dev_dbg(port->dev, ">ni16550_config_rs485\n");

	/* "rs485" should be given to us non-NULL. */
	BUG_ON(rs485 == NULL);

	pcr = port->serial_in(port, NI16550_PCR_OFFSET);
	pcr &= ~NI16550_PCR_WIRE_MODE_MASK;

	if (rs485->flags & SER_RS485_ENABLED) {
		/* RS-485 */
		if ((rs485->flags & SER_RS485_RX_DURING_TX) &&
		    (rs485->flags & SER_RS485_RTS_ON_SEND)) {
			dev_dbg(port->dev, "Invalid 2-wire mode\n");
			return -EINVAL;
		}

		if (rs485->flags & SER_RS485_RX_DURING_TX) {
			/* Echo */
			dev_vdbg(port->dev, "2-wire DTR with echo\n");
			pcr |= NI16550_PCR_ECHO_RS485;
		} else {
			/* Auto or DTR */
			if (rs485->flags & SER_RS485_RTS_ON_SEND) {
				/* Auto */
				dev_vdbg(port->dev, "2-wire Auto\n");
				pcr |= NI16550_PCR_AUTO_RS485;
			} else {
				/* DTR-controlled */
				/* No Echo */
				dev_vdbg(port->dev, "2-wire DTR no echo\n");
				pcr |= NI16550_PCR_DTR_RS485;
			}
		}
	} else {
		/* RS-422 */
		dev_vdbg(port->dev, "4-wire\n");
		pcr |= NI16550_PCR_RS422;
	}

	dev_dbg(port->dev, "write pcr: 0x%08x\n", pcr);
	port->serial_out(port, NI16550_PCR_OFFSET, pcr);

	/* Update the cache. */
	port->rs485 = *rs485;

	dev_dbg(port->dev, "<ni16550_config_rs485\n");
	return 0;
}

static bool is_rs232_mode(struct uart_8250_port *up)
{
	uint8_t pmr = serial_in(up, NI16550_PMR_OFFSET);

	/* If the PMR is not implemented, then by default NI UARTs are
	 * connected to RS-485 transceivers
	 */
	if ((pmr & NI16550_PMR_CAP_MASK) == NI16550_PMR_NOT_IMPL)
		return false;

	if ((pmr & NI16550_PMR_CAP_MASK) == NI16550_PMR_CAP_DUAL)
		/* If the port is dual-mode capable, then read the mode bit
		 * to know the current mode
		 */
		return ((pmr & NI16550_PMR_MODE_MASK)
					== NI16550_PMR_MODE_RS232);
	else
		/* If it is not dual-mode capable, then decide based on the
		 * capability
		 */
		return ((pmr & NI16550_PMR_CAP_MASK) == NI16550_PMR_CAP_RS232);
}

static void ni16550_config_prescaler(struct uart_8250_port *up,
				     uint8_t prescaler)
{
	/* Page in the Enhanced Mode Registers
	 * Sets EFR[4] for Enhanced Mode.
	 */
	uint8_t lcr_value;
	uint8_t efr_value;

	lcr_value = serial_in(up, UART_LCR);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	efr_value = serial_in(up, UART_EFR);
	efr_value |= UART_EFR_ECB;

	serial_out(up, UART_EFR, efr_value);

	/* Page out the Enhanced Mode Registers */
	serial_out(up, UART_LCR, lcr_value);

	/* Set prescaler to CPR register. */
	serial_out(up, UART_SCR, UART_CPR);
	serial_out(up, UART_ICR, prescaler);
}

static void ni16550_port_setup(struct uart_port *port)
{
	port->rs485_config = &ni16550_config_rs485;
	/* The hardware comes up by default in 2-wire auto mode and we
	 * set the flags to represent that
	 */
	port->rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
}

static int ni16550_port_startup(struct uart_port *port)
{
	int ret;

	ret = serial8250_do_startup(port);
	if (!ret)
		return ret;

	if (port->rs485_config)
		ret = ni16550_enable_transceivers(port);

	return ret;
}

static void ni16550_port_shutdown(struct uart_port *port)
{
	if (port->rs485_config)
		ni16550_disable_transceivers(port);

	serial8250_do_shutdown(port);
}

static const struct uart_ops ni16550_uart_ops = {
	.startup	= ni16550_port_startup,
	.shutdown	= ni16550_port_shutdown,
};

static int ni16550_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uart_8250_port uart = {};
	struct ni16550_data *data;
	const struct ni16550_device_info *info;
	struct resource *regs;
	int ret = 0;
	int irq;
	int rs232_property = 0;
	unsigned prescaler;
	const char *transceiver;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&uart.port.lock);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	if ((regs = platform_get_resource(pdev, IORESOURCE_IO, 0))) {
		uart.port.iotype = UPIO_PORT;
		uart.port.iobase = regs->start;
	} else if ((regs = platform_get_resource(pdev, IORESOURCE_MEM, 0))) {
		uart.port.iotype  = UPIO_MEM;
		uart.port.mapbase = regs->start;
		uart.port.mapsize = resource_size(regs);
		uart.port.flags   |= UPF_IOREMAP;

		uart.port.membase = devm_ioremap(dev, uart.port.mapbase,
						 uart.port.mapsize);
		if (!uart.port.membase)
			return -ENOMEM;
	} else {
		dev_err(dev, "no registers defined\n");
		return -EINVAL;
	}

	/* early setup so that serial_in()/serial_out() work */
	serial8250_set_defaults(&uart);

	info = device_get_match_data(dev);

	uart.port.dev		= dev;
	uart.port.irq		= irq;
	uart.port.irqflags	= IRQF_SHARED;
	uart.port.flags		|= UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF
					| UPF_FIXED_PORT | UPF_FIXED_TYPE;
	uart.port.type		= info->port_type;
	uart.port.ops		= &ni16550_uart_ops;

	/*
	 * OF device-tree and NIC7A69 ACPI can declare clock-frequency,
	 * but may be missing for other instantiations, so this is optional.
	 * If present, override what we've defined staticly.
	 */
	if (info->uartclk)
		uart.port.uartclk = info->uartclk;
	device_property_read_u32(dev, "clock-frequency", &uart.port.uartclk);
	if (!uart.port.uartclk) {
		dev_err(dev, "unable to determine clock frequency!\n");
		return -ENODEV;
	}

	if (info->prescaler)
		prescaler = info->prescaler;
	device_property_read_u32(dev, "clock-prescaler", &prescaler);

	if (prescaler != 0) {
		uart.mcr_force = UART_MCR_CLKSEL;
		ni16550_config_prescaler(&uart, (uint8_t)prescaler);
	}

	/*
	 * Similarly, "transceiver" might not be present. If it is not present,
	 * then this is probably RS-485 (unless PMR is implemented).
	 */
	if (!device_property_read_string(dev, "transceiver", &transceiver)) {
		rs232_property = strncmp(transceiver, "RS-232", 6) == 0;
	}

	/*
	 * NI UARTs may be connected to RS-485 or RS-232 transceivers,
	 * depending on the ACPI 'transceiver' property and whether or
	 * not the PMR is implemented. If the PMR is implemented and
	 * the port is in RS-232 mode, register as a standard 8250 port
	 * and print about it.
	 */
	if ((info->flags & NI_CAP_PMR) && is_rs232_mode(&uart)) {
		if (uart.port.iotype == UPIO_PORT)
			pr_info("NI 16550 at I/O 0x%x (irq = %d) is dual-mode capable and is in RS-232 mode\n",
				(unsigned int)uart.port.iobase,
				uart.port.irq);
		else
			pr_info("NI 16550 at MMIO 0x%llx (irq = %d) is dual-mode capable and is in RS-232 mode\n",
				(unsigned long long)uart.port.mapbase,
				uart.port.irq);
	} else if (!rs232_property) {
		/*
		 * Either the PMR is implemented and set to RS-485 mode
		 * or it's not implemented and the 'transceiver' ACPI
		 * property is 'RS-485';
		 */
		ni16550_port_setup(&uart.port);
	}

	ret = serial8250_register_8250_port(&uart);
	if (ret < 0)
		return ret;

	data->line = ret;

	platform_set_drvdata(pdev, data);

	return 0;
}

static int ni16550_remove(struct platform_device *pdev)
{
	struct ni16550_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);
	return 0;
}

/* NI 16550 (with 16-byte FIFOs) */
static const struct ni16550_device_info ni16550_fifo16 = {
	.port_type = PORT_NI16550_F16,
};

/* NI 16550 (with 128-byte FIFOs) */
static const struct ni16550_device_info ni16550_fifo128 = {
	.port_type = PORT_NI16550_F128,
};

static const struct of_device_id ni16550_of_match[] = {
	{ .compatible = "ni,ni16550-fifo16",
		.data = &ni16550_fifo16, },
	{ .compatible = "ni,ni16550-fifo128",
		.data = &ni16550_fifo128, },
	{ },
};
MODULE_DEVICE_TABLE(of, ni16550_of_match);

/* NI 16550 RS-485 Interface */
static const struct ni16550_device_info nic7750 = {
	.uartclk = 33333333,
	.port_type = PORT_NI16550_F128,
};

/* NI CVS-145x RS-485 Interface */
static const struct ni16550_device_info nic7772 = {
	.uartclk = 1843200,
	.port_type = PORT_NI16550_F16,
	.flags = NI_CAP_PMR,
};

/* NI cRIO-904x RS-485 Interface */
static const struct ni16550_device_info nic792b = {
	/* Sets UART clock rate to 22.222 MHz with 1.125 prescale */
	.uartclk = 25000000,
	.prescaler = NI16550_CPR_PRESCALE_1x125,
	.port_type = PORT_NI16550_F128,
};

/* NI sbRIO 96x8 RS-232/485 Interfaces */
static const struct ni16550_device_info nic7a69 = {
	/* Set UART clock rate to 29.629 MHz with 1.125 prescale */
	.uartclk = 29629629,
	.prescaler = NI16550_CPR_PRESCALE_1x125,
	.port_type = PORT_NI16550_F128,
};

static const struct acpi_device_id ni16550_acpi_match[] = {
	{ "NIC7750",	(kernel_ulong_t)&nic7750 },
	{ "NIC7772",	(kernel_ulong_t)&nic7772 },
	{ "NIC792B",	(kernel_ulong_t)&nic792b },
	{ "NIC7A69",	(kernel_ulong_t)&nic7a69 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, ni16550_acpi_match);

static struct platform_driver ni16550_driver = {
	.driver = {
		.name = "ni16550",
		.of_match_table = ni16550_of_match,
		.acpi_match_table = ACPI_PTR(ni16550_acpi_match),
	},
	.probe = ni16550_probe,
	.remove = ni16550_remove,
};

module_platform_driver(ni16550_driver);

MODULE_AUTHOR("Jaeden Amero <jaeden.amero@ni.com>");
MODULE_AUTHOR("Karthik Manamcheri <karthik.manamcheri@ni.com>");
MODULE_DESCRIPTION("NI 16550 Driver");
MODULE_LICENSE("GPL v2");
