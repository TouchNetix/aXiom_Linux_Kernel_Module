# aXiom Linux Kernel Module

## Synopsis

For first time use, `init_env.sh` compiles the source files and creates kernel objects (`.ko`). The script will then load the specified kernel module.

```bash
sudo ./init_env.sh [spi/i2c]
```

The `installmodule.sh` script will unload any previously loaded aXiom modules before loading the specified kernel module.

```bash
sudo ./installmodule.sh [spi/i2c]
```

## Description

The aXiom Linux Kernel module is a driver used to interface aXiom devices with the Linux Input Subsystem. It decodes u41 target reports and u46 force reports received from and converts the reports into messages for the Linux input subsystem. The reports can be read from aXiom via I2C or SPI. All aXiom are recieved by the kernel module, however only u41 and u46 are processed. All other reports are logged in `dmesg`.

## Prerequisites

### System

* Root/sudo permissions for the host machine
* Knowledge of the kernel version on the system and the appropriate kernel headers installed

### Hardware

* aXiom EVK or Control Board
  * A suitable firmware version must be loaded onto this
  * The configuration loaded onto the device needs
    * u41 target reports enabled, requires a touch sensor
    * u46 force reports enabled, requires a system with force sensing capabilities

### Kernel Versions and Testing

This driver has been tested with Kernels versions 5.04, 5.11, and 5.15.

This has been tested in the following systems:

* Raspberry Pi (3 and 4)
  * I2C and SPI have been tested

## Software Repository

In this repository you will find:

* axiom_core.c
* axiom_core.h
* axiom_i2c.c
* axiom_spi.c
* axiom_i2c_overlay.dts
* axiom_spi_overlay.dts
* init_env.sh
* installmodule.sh
* Makefile

## User Configurable Options

### axiom_core.h

#### AXIOM_USE_TOUCHSCREEN_INTERFACE

* Having this option disabled allows for the device to be registered as a mouse instead of a touchscreen. This allows for more complex gestures, such as specific reporting for multiple taps (i.e. tapping with 5 fingers at once is reported as a 'quinttap')

#### AXIOM_USE_KERNEL_SLOT_ASSIGNMENT

* The kernel slot assignment is required if multiple touch devices are being used within the same system - this allows for the kernel to track the slots for the different devices.

#### U46_ENABLE_RAW_FORCE_DATA

* This option allows for the raw force data for each of up to 4 channels on the aXiom device to be sent as miscellaneous raw input events. This option requires a force compatible sensor and a device configuration with u46 reports enabled.
* Please note that the raw force values are signed, but sent as unsigned values.

### Individual Files

#### DEBUG

* Enables debug messages for that file to be sent via dmesg
  * Debug messages viewable using `sudo dmesg -w`
  * This must be declared at the top of the file

## Module Installation

### Preparations to compile the driver

#### Raspberry Pi

If using a raspberry Pi, either the native kernel headers can be used or the kernel can be checked out from git.

```bash
# native headers
$ sudo apt install raspberrypi-kernel-headers
```

For a Raspberry Pi based system get a copy of Kernel 5.4:

```bash
# git checkout
wget https://github.com/raspberrypi/linux/tree/rpi-5.4.y
unzip rpi-5.4.y
```

For this to yield a viable build structure it need to be configured and partially compiled eg:

```bash
# For Raspberry Pi:
cd linux
KERNEL=kernel7
make bcm2709_defconfig

# Start a compile, but wait ~1 minute, then cancel via ctrl+c
make -j4 zImage modules dtbs
```

#### Other Linux-based systems

With the provided files in the desired location, ensure the makefile is pointing to the location of the kernel header files. By default this is set to:

```bash
KERNEL_LOC=/lib/modules/$(shell uname -r)/build/
```

The above should work for the majority of systems, however if this is not the case then the makefile should be modified. The following example is for 5.15.0-53:

```bash
KERNEL_LOC=/usr/src/linux-headers-5.15.0-53-generic'
```

### Compilation

#### Raspberry Pi Compilation

To compile for 5.04 the following macro must be defined in *axiom_core.h*:

```c
// at around line 50
#define __AXIOM_KERNEL_5_4
```

#### Other linux-based systems

There are two options for compilation of the driver - manual, or using the provided helper scripts.
To manually compile the files:

```bash
# As simple as...
$ make -j4 all
```

Using the helper script 'init_env.sh' will both compile the files and install the module for the specified comms method:

```bash
# As simple as...
$ ./init_env.sh [spi/i2c]
```

Please note - if using either of the helper scripts, they need to be given executable permissions.

### Installation

The module must be installed on a system using the same kernel version as it was compiled against.
If `./init_env.sh` was used in the compilation step, the module has already been installed.

The general flow is as follows:

* Un-load any axiom drivers (if loaded).
* Load the axiom_spi_drv or axiom_i2c_drv module depending on your application.

This can be done from the command line using the following instructions, or using one of the provided helper scripts.

To manually install:

```bash
# Make sure to run as root or by using "sudo" as necessary
# unload any pre-existing axiom modules
$ sudo rmmod axiom_spi_drv.ko; sudo rmmod axiom axiom_i2c_drv.ko

# load the recently compiled modules:
$ sudo insmod axiom_spi_drv.ko

# You can see any messages from the driver via dmesg:
$ sudo dmesg -w
```

Using the helper script:

```bash
# unloads any loaded axiom modules, then loads the specified module including any associated device tree overlays
$ sudo ./installmodule.sh [spi/i2c]
```

### I2C

#### I2C Connection

The I2C address must be set by the I2CADDR jumpers on the EVK.
The default address is 0x66 and the alternative address is 0x67. The axiom_i2c_overlay.dts file must be set to match the chosen address.
Note: If the "I2CADDR" header is left unpopulated, then the aXiom will respond on address 0x67.
The `i2cdetect` utility can be used to discover the address of the connected device.

You may need to hard reboot the Raspberry Pi after making any of the above changes.

#### I2C Notes

By default the Raspberry Pi I2C baud rate is 100kHz. Although the aXiom will work at this rate, but it will result in lost reports. The aXiom works best at 400kHz baud rate, so the following change to the i2c_arm dtparam in the /boot/config.txt file must be made to avoid this:

```bash
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

Uses Raspberry Pi GPIO24 as an interrupt to tell the kernel when new reports are
available to consume.

### SPI

#### SPI Connection

Raspberry Pi bus SPI bus 0 and CE0 are configured to be used in the device tree
overlay.

#### SPI Notes

Uses Raspberry Pi GPIO24 as an interrupt to tell the kernel when new reports are
available to consume.

## Module Usage

### Module Initialisation

* Upon module installation, or device connection (if the module was already installed), information about the aXiom device is retrieved
  * This includes populating the usage table
* The user configurable options are checked
* The device is registered with the input subsystem and limits for input events are set

### Report Processing

* When a report is received and ready to be consumed, an interrupt will notify the system.
  * The information held in the report is read by the selected comms method - the process varies between I2C and SPI
* Before the report is processed, the information held in the report is validated
  * Length - reports with zero length are discarded.
  * CRC - the CRC of the report is calculated and compared to the CRC calculated by aXiom. If they do not match, the report is discarded.
* The usage responsible for sending the report is then checked
  * u41 reports are always processed, with the touch data sent to the input subsystem
  * u46 reports are processed if the setting is enabled in 'axiom_core.h' and the raw force data sent to the input subsystem
  * Other reports are not processed, but the raw report data is printed as a 'dev_info' message

## Reports

### u41

u41 contains Touch, Proximity, and Hover information for up to 10 different targets. Each target is processed individually and a multitouch sync frame is sent after the information for each target is extracted. The steps for each target are listed below:

* x, y, and z data is read from the report buffer
* Using the 'z' data, the type of touch is discerned.
  * A positive z value indicates a force (pressure) event
  * A z value of 0 indicates a touch event, contact has been made with the sensor but no force applied
  * A negative z value indicates a hover (distance) event
  * A z value of `PROX_LEVEL` (-128) indicates a proximity event
* The co-ordinate data and touch type is compared to the previous data from the last report for the relevant target.
  * If the co-ordinates have not changed then a report is **not** sent to the input subsystem.
  * The data will not be sent, even if the option to sent u41 report every frame is enabled in the configuration file.
* Touch data is assigned to the relevant supported input event types and sent to the input subsystem.
  * Any unused supported input events (such as pressure if the target is a distance event) is set to 0
  * If the target is the first reported contact, a button press is also reported
  * The slots used to report the data depends on whether kernel slot assignment was enabled
* After the information has been processed, it is stored to be used as a comparison for the data from the next frame

When each target has been processed, an overall input sync is sent to the input subsystem.

### u46

u46 contains raw force data for up to 4 different channels on the device. The processing of these reports is not enabled by default and must be enabled prior to compilation in axiom_core.h. Additionally, for these reports to contain any data, a force compatible device and sensor must be connected with u46 reports enabled in the device configuration file.

The u46 data is sent to the input subsystem as a 'MSC_RAW' (miscellaneous raw event), and the channel number the data was read from is prepended to the value as such:

* 00123
* 10123
* 20123
* 30123

It must be noted that force values can be positive or negative, however the report from aXiom contains only unsigned values. If these values are being used somewhere else then this must be accounted for.

## Interfacing with aXiom Facilities Code

A selection of python helper scripts have been developed by TouchNetix which can be used in a non-Windows environment. These scripts allow for basic interfacing with the aXiom device such as loading device configurations or firmware versions. With the kernel module installed, these scripts can be run when using I2C, SPI or USB. Despite USB and I2C working, it is not recommended to be upgrading firmware or configurations 'on the fly'.

### Available Scripts

* axcfg.py
  * Loads a new configuration file onto the device
  * Only supports .th2cfgbin files
* axfw.py
  * Loads a new firmware version onto the device
  * Accepts .alc and .axfw files
* axtbp.py
  * USB only
  * Bridge info

Sudo apt install python3-pip to install pip on linux, sudo pip install hid, sudo apt install libhidapi-hidraw0

## Introducing Additional Functionality

The provided code processes u41 and u46 reports, sending the information to the input subsystem and there are no plans for expanding this functionality. The purpose of the code is to provide a base for communication between aXiom and the Linux input subsystem, and development of additional features is encouraged.

### Accessing Usages

During device initialisation, the usage table is populated, which contains information about each usage including the address. A function in 'axiom_core.c' called 'usage_to_target_address' has been provided to translate a usage into an address. There is a 'axiom_read_usage' and 'axiom_write_usage' for each communication method.

### Reading Different Reports

The function 'axiom_process_report' in 'axiom_core.c' contains a switch case statement which dictates what happens to different reports. In order to process additional reports the usage number needs to be added to this statement, with the processing instructions.
Information about the structure of the different reports to enable development of a function to process reports is available upon request.

### Supported Input Events

* ABS_X
* ABS_Y
* ABS_MT_POSITION_X
* ABS_MT_POSITION_Y
* ABS_MT_TOOL_TYPE
* ABS_MT_DISTANCE
* ABS_PRESSURE
* EV_KEY
  * BTN_TOOL_PEN
  * BTN_LEFT (when one click with touch 0)
* EV_REL
* EV_MSC
  * MSC_RAW - raw u46 force data

## Contact

If any issues arise while using the driver, please contact TouchNetix at: support@touchnetix.com
