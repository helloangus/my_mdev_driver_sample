obj-m := host_mdev_sched.o
PWD := $(shell pwd)

KERNEL_INFO := $(shell uname -r)
KDIR := /usr/src/linux-headers-$(KERNEL_INFO)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	rm -f *.o *.ko *.mod.* Module.symvers
	$(MAKE) -C $(KDIR) M=$(PWD) clean