obj-m := mtty.o

PWD := $(shell pwd)
KERNEL_INFO := $(shell uname -r)
KERNEL_LIB := /usr/src/linux-headers-$(KERNEL_INFO)

all:
	make -C $(KERNEL_LIB) M=$(PWD) modules
clean:
	make -C $(KERNEL_LIB) M=$(PWD) clean

