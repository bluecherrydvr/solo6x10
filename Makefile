KVERS := $(shell uname -r)
MODDIR = /lib/modules/$(KVERS)

KERNELDIR = $(MODDIR)/build
KERNELSRC = $(MODDIR)/source

export KERNELSRC

solo6x10-edge-y := solo6x10-core.o solo6x10-i2c.o solo6x10-p2m.o \
		   solo6x10-v4l2.o solo6x10-tw28.o solo6x10-gpio.o \
		   solo6x10-disp.o solo6x10-enc.o solo6x10-v4l2-enc.o \
		   solo6x10-g723.o solo6x10-eeprom.o

obj-m		:= solo6x10-edge.o

# Must build modpost if not found
ifeq ($(wildcard $(KERNELDIR)/scripts/mod/modpost),)
modules: scripts
scripts: FORCE
	$(MAKE) $(MAKEARGS) -C $(KERNELDIR) $@
endif

modules modules_install clean: FORCE
	$(MAKE) $(MAKEARGS) -C $(KERNELDIR) M=$(CURDIR) $@

install: modules_install FORCE
modules_install: blacklist-mainline FORCE

blacklist-mainline: FORCE
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

-include $M/mk/videobuf.mk
endif

FORCE:

# For my local crud
-include make.extras
