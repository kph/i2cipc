ifneq ($(KERNELRELEASE),)

obj-m += i2c-initiator-stream.o

else

KVERSION ?= `uname -r`
KDIR ?= /lib/modules/$(KVERSION)/build

i2c-initiator-stream-kmod:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	rm -f *.o *.a *.cmd *.ko .i2c-initiator-stream*.cmd Module.symvers modules.order i2c-initiator-stream.mod i2c-initiator-stream.mod.c

.PHONY: i2c-initiator-stream-kmod clean

endif
