KERNELDIR	= /lib/modules/$(shell uname -r)

solo6010-objs	:= solo6010-core.o solo6010-i2c.o solo6010-p2m.o \
		   solo6010-v4l2.o solo6010-tw28.o solo6010-gpio.o \
		   solo6010-disp.o solo6010-enc.o solo6010-v4l2-enc.o
obj-m		:= solo6010.o

all:
	$(MAKE) -C $(KERNELDIR)/build M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KERNELDIR)/build M=$(shell pwd) clean
