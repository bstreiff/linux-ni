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

/* flags for ACPI match */
#define NI_16BYTE_FIFO		0x0001
#define NI_CAP_PMR		0x0002
#define NI_CLK_33333333		0x0004
#define NI_CPR_CLK_25000000	0x0008
#define NI_CPR_CLK_33333333	0x0010

struct ni16550_data {
	int			line;
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

bool is_rs232_mode(unsigned long iobase)
{
	uint8_t pmr = inb(iobase + NI16550_PMR_OFFSET);

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

void ni16550_config_prescaler(unsigned long iobase, uint8_t prescaler)
{
	/* Page in the Enhanced Mode Registers
	 * Sets EFR[4] for Enhanced Mode.
	 */
	uint8_t lcr_value;
	uint8_t efr_value;

	lcr_value = inb(iobase + UART_LCR);
	outb(UART_LCR_CONF_MODE_B, iobase + UART_LCR);

	efr_value = inb(iobase + UART_EFR);
	efr_value |= UART_EFR_ECB;

	outb(efr_value, iobase + UART_EFR);

	/* Page out the Enhanced Mode Registers */
	outb(lcr_value, iobase + UART_LCR);

	/* Set prescaler to CPR register. */
	outb(UART_CPR, iobase + UART_SCR);
	outb(prescaler, iobase + UART_ICR);
}

static struct txvr_ops ni16550_txvr_ops = {
	.enable_transceivers = ni16550_enable_transceivers,
	.disable_transceivers = ni16550_disable_transceivers,
};

void ni16550_port_setup(struct uart_port *port)
{
	port->txvr_ops = &ni16550_txvr_ops;
	port->rs485_config = &ni16550_config_rs485;
	/* The hardware comes up by default in 2-wire auto mode and we
	 * set the flags to represent that
	 */
	port->rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
}

static int ni16550_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct uart_8250_port uart = {};
	struct ni16550_data *data;
	struct resource *regs;
	int ret = 0;
	int irq;
	int rs232_property = 0;
	long flags;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(dev, "no registers defined\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	uart.port.membase = devm_ioremap(dev, regs->start,
					 resource_size(regs));
	if (!uart.port.membase)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&uart.port.lock);
	uart.port.irq		= irq;
	uart.port.irqflags	= IRQF_SHARED;
	uart.port.iotype	= UPIO_MEM;
	uart.port.flags		= UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF
						| UPF_FIXED_PORT | UPF_IOREMAP
						| UPF_FIXED_TYPE;
	uart.port.mapbase	= regs->start;
	uart.port.mapsize	= resource_size(regs);
	uart.port.dev		= dev;

	flags = (long)device_get_match_data(dev);

	if (flags & NI_CLK_33333333)
		uart.port.uartclk = 33333333;
	if (flags & NI_CPR_CLK_25000000) {
		/* Sets UART clock rate to 22.222 MHz with 1.125 prescale */
		uart.port.uartclk = 22222222;
		uart.mcr_force = UART_MCR_CLKSEL;
		ni16550_config_prescaler(uart.port.iobase,
					 NI16550_CPR_PRESCALE_1x125);
	}
	if (flags & NI_CPR_CLK_33333333) {
		const char *transceiver;

		/* Set UART clock rate to 29.629 MHz with 1.125 prescale */
		uart.port.uartclk = 29629629;
		uart.mcr_force = UART_MCR_CLKSEL;
		ni16550_config_prescaler(uart.port.iobase,
					 NI16550_CPR_PRESCALE_1x125);

		/* TODO: why is this only in this case? */
		/* we want this for the OF-enumerated case too */
		if (device_property_read_string(dev, "transceiver",
						&transceiver)) {
			dev_warn(dev, "no transceiver property set\n");
			ret = -EINVAL;
		}

		rs232_property = strncmp(transceiver, "RS-232", 6) == 0;
	}

	if (flags & NI_16BYTE_FIFO)
		uart.port.type = PORT_NI16550_F16;
	else
		uart.port.type = PORT_NI16550_F128;

	/*
	 * NI UARTs may be connected to RS-485 or RS-232 transceivers,
	 * depending on the ACPI 'transceiver' property and whether or
	 * not the PMR is implemented. If the PMR is implemented and
	 * the port is in RS-232 mode, register as a standard 8250 port
	 * and print about it.
	 */
	if ((flags & NI_CAP_PMR) && is_rs232_mode(uart.port.iobase))
		pr_info("NI 16550 at I/O 0x%x (irq = %d) is dual-mode capable and is in RS-232 mode\n",
				 (unsigned int)uart.port.iobase,
				 uart.port.irq);
	else if (!rs232_property)
		/*
		 * Either the PMR is implemented and set to RS-485 mode
		 * or it's not implemented and the 'transceiver' ACPI
		 * property is 'RS-485';
		 */
		ni16550_port_setup(&uart.port);

	platform_set_drvdata(pdev, data);

	data->line = serial8250_register_8250_port(&uart);
	if (data->line < 0)
		return data->line;

	return 0;
}

static int ni16550_remove(struct platform_device *pdev)
{
	struct ni16550_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);
	return 0;
}

static const struct of_device_id ni16550_of_match[] = {
	{ .compatible = "ni,ni16550-fifo16",
		.data = (void *)NI_16BYTE_FIFO, },
	{ .compatible = "ni,ni16550-fifo128",
		.data = (void *)0 },
	{ },
};
MODULE_DEVICE_TABLE(of, ni16550_of_match);

static const struct acpi_device_id ni16550_acpi_match[] = {
	{ "NIC7750",	NI_CLK_33333333 },
	{ "NIC7772",	NI_CAP_PMR | NI_16BYTE_FIFO },
	{ "NIC792B",	NI_CPR_CLK_25000000 },
	{ "NIC7A69",	NI_CPR_CLK_33333333 },
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
