KERNELDIR	= /lib/modules/$(shell uname -r)/build

# For my local crud
-include make.extras

solo6x10-objs	:= solo6010-core.o solo6010-i2c.o solo6010-p2m.o \
		   solo6010-v4l2.o solo6010-tw28.o solo6010-gpio.o \
		   solo6010-disp.o solo6010-enc.o solo6010-v4l2-enc.o \
		   solo6010-g723.o solo6010-eeprom.o

# For when the kernel isn't compiled with it
ifeq ($(CONFIG_VIDEOBUF_DMA_CONTIG),)
solo6x10-objs += videobuf-dma-contig.o
solo6x10-objs += videobuf-dma-sg.o
solo6x10-objs += videobuf-core.o
endif

obj-m		:= solo6x10.o


all:
	$(MAKE) $(MAKEARGS) -C $(KERNELDIR) M=$(shell pwd) modules

install:
	$(MAKE) $(MAKEARGS) -C $(KERNELDIR) M=$(shell pwd) modules_install

clean:
	$(MAKE) $(MAKEARGS) -C $(KERNELDIR) M=$(shell pwd) clean
	rm -f Module.markers modules.order
