obj-m += rtl8169.o
ccflags-y := -std=gnu99 -Wno-declaration-after-statement
.PHONY: all clean

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean