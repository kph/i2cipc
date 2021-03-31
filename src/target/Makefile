ifneq ($(KERNELRELEASE),)

obj-m += i2c-target-stream.o

else

KVERSION ?= `uname -r`
KDIR ?= /lib/modules/$(KVERSION)/build

i2c-target-stream-kmod:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	rm -f *.o *.a *.cmd *.ko .i2c-target-stream*.cmd Module.symvers modules.order i2c-target-stream.mod i2c-target-stream.mod.c

.PHONY: i2c-target-stream-kmod clean

endif
