KVERS := $(shell uname -r)
MODDIR = /lib/modules/$(KVERS)

KERNELDIR = $(MODDIR)/build
KERNELSRC = $(MODDIR)/source

solo6x10-edge-y := core.o i2c.o p2m.o \
		   v4l2.o tw28.o gpio.o \
		   disp.o enc.o v4l2-enc.o \
		   g723.o eeprom.o

obj-m		:= solo6x10-edge.o

modules modules_install clean: FORCE
	$(MAKE) $(MAKEARGS) -C $(KERNELDIR) M=$(CURDIR) $@

install: modules_install FORCE
	$(if $(KBUILD_VERBOSE:1=),@echo '  BLACKLIST solo6x10')
	$(Q)echo blacklist solo6x10 > /etc/modprobe.d/solo6x10.conf

clean: clean_local

clean_local: FORCE
	rm -f Module.markers modules.order \
	      videobuf-dma-contig.c videobuf-dma-sg.c \
	      videobuf-dma-contig.c.in videobuf-dma-sg.c.in


ifneq ($(CONFIG_VIDEOBUF_DMA_CONTIG)$(CONFIG_VIDEOBUF_DMA_SG),yy)
# For when the kernel isn't compiled with it
solo6x10-edge-y$(CONFIG_VIDEOBUF_DMA_CONTIG)	+= videobuf-dma-contig.o
solo6x10-edge-y$(CONFIG_VIDEOBUF_DMA_SG)	+= videobuf-dma-sg.o

-include $M/videobuf.mk
endif

FORCE:

# For my local crud
-include make.extras
