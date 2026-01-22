obj-m += axiom_spi_drv.o
obj-m += axiom_i2c_drv.o

KERNEL_LOC=/lib/modules/$(shell uname -r)/build/

axiom_spi_drv-objs := axiom_core.o axiom_spi.o
axiom_i2c_drv-objs := axiom_core.o axiom_i2c.o

all:
	$(MAKE) -C $(KERNEL_LOC) M=$(PWD) modules

install:
	$(MAKE) -C $(KERNEL_LOC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_LOC) M=$(PWD) clean