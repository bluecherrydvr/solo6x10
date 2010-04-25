KERNELDIR	= /lib/modules/$(shell uname -r)/build

solo6010-objs	:= solo6010-core.o solo6010-i2c.o solo6010-p2m.o \
		   solo6010-v4l2.o solo6010-tw28.o solo6010-gpio.o \
		   solo6010-disp.o solo6010-enc.o solo6010-v4l2-enc.o \
		   solo6010-g723.o solo6010-g723-dec.o

ifeq ($(CONFIG_VIDEOBUF_DMA_CONTIG),)
solo6010-objs += videobuf-dma-contig.o
endif

obj-m		:= solo6010.o

all:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(shell pwd) clean
	rm -f Module.markers modules.order
