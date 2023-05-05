#!/bin/bash
#
# Bash script that unloads and reloads the required kernel module,
# just define on the command line which module to load.
#
# Usage:
#		./installmodule.sh [usb/spi/i2c]
#

if [[ $(lsmod | grep axiom_usb | wc -l) != "0" ]]; then
	echo "Removing axiom kernel USB module..."
	sudo rmmod axiom_usb
fi
if [[ $(lsmod | grep axiom_spi| wc -l) != "0" ]]; then
	echo "Removing axiom SPI module..."
	sudo rmmod axiom_spi
fi
if [[ $(lsmod | grep axiom_i2c| wc -l) != "0" ]]; then
	echo "Removing axiom I2C module..."
	sudo rmmod axiom_i2c
fi
if [[ $(dtc -I fs /proc/device-tree | grep axiom | wc -l) != "0" ]]; then
	echo "Unloading Device tree..."
	sudo dtoverlay -r axiom_spi_overlay
	sudo dtoverlay -r axiom_i2c_overlay
fi
if [[ $(dtc -I fs /proc/device-tree | grep axiom | wc -l) != "0" ]]; then
	echo "OOPS! couldn't unload axiom from device tree!"
	exit 1
fi

if [ "${1}" == "usb" ]; then 
	echo Installing axiom USB kernel modules...
	sudo insmod axiom_usb.ko
else
	if [ "${1}" == "spi" ]; then 
		echo Installing axiom SPI device tree and kernel modules...
		sudo dtoverlay axiom_spi_overlay.dtbo
		sudo insmod axiom_spi.ko
	else
		if [ "${1}" == "i2c" ]; then 
			echo Installing axiom I2C device tree and kernel modules...
			sudo dtoverlay axiom_i2c_overlay.dtbo
			sudo insmod axiom_i2c.ko
		else
			echo Bad or missing parameter. Expecting usb, spi or i2c
			exit 1
		fi
	fi
fi

if [ "${1}" == "usb" ]; then
	if [[ $(lsmod | grep axiom_usb | wc -l) != "0" ]]; then
		echo Axiom USB kernel module loaded
	fi 
else
	if [[ $(dtc -I fs /proc/device-tree | grep axiom | wc -l) != "0" ]]; then
		echo Device tree loaded
	else
		echo Failed to load axiom overlay into device tree!
		exit 1
	fi
	if [[ $(lsmod | grep axiom_spi| wc -l) != "0" ]]; then
		echo Axiom SPI kernel module loaded
	fi
	if [[ $(lsmod | grep axiom_i2c| wc -l) != "0" ]]; then
		echo Axiom I2C kernel module loaded
	fi
fi
