obj-m += axiom_usb.o
obj-m += axiom_spi.o
obj-m += axiom_i2c.o

KERNEL_LOC=/lib/modules/$(shell uname -r)/build/

axiom_usb-objs := axiom_core.o axiom_usb_comms.o
axiom_spi-objs := axiom_core.o axiom_spi_comms.o
axiom_i2c-objs := axiom_core.o axiom_i2c_comms.o

all:
	$(MAKE) -C $(KERNEL_LOC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_LOC) M=$(PWD) clean
