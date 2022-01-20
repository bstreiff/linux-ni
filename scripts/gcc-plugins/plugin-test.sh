#!/bin/sh

CC_INV=$($* -print-file-name=plugin)
LOOKING_FOR=${CC_INV}/include/plugin-version.h
test -e ${LOOKING_FOR}
RET=$?

echo "looking for ${LOOKING_FOR}: ret = ${RET}" >> /tmp/kernel-debug-log.txt

exit $RET
