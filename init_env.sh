#!/bin/bash
#
# Bash script that compiles all the kernel modules, then loads the one required,
# just define on the command line which module to load.
# This script also makes sure the other scripts are marked executable.
#
# Usage:
#		./init_env.sh [usb/spi/i2c]
#

echo Compiling source...
make -j4 all
rc=$?
if [ ! ${rc} -eq 0 ]; then
	echo make failed with error ${rc}
	exit ${rc}
fi
dtc -I dts -O dtb -o axiom_spi_overlay.dtbo axiom_spi_overlay.dts
rc=$?
if [ ! ${rc} -eq 0 ]; then
	echo Failed compiling SPI dtc overlay with error ${rc}
	exit ${rc}
fi
dtc -I dts -O dtb -o axiom_i2c_overlay.dtbo axiom_i2c_overlay.dts
rc=$?
if [ ! ${rc} -eq 0 ]; then
	echo Failed compiling I2C dtc overlay with error ${rc}
	exit ${rc}
fi

if [ ! -z ${1} ]; then
	./installmodule.sh ${1}
fi