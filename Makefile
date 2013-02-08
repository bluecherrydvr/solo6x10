KVERS := $(shell uname -r)
MODDIR = /lib/modules/$(KVERS)

KERNELDIR = $(MODDIR)/build
KERNELSRC = $(MODDIR)/source

solo6x10-edge-objs := solo6010-core.o solo6010-i2c.o solo6010-p2m.o \
		   solo6010-v4l2.o solo6010-tw28.o solo6010-gpio.o \
		   solo6010-disp.o solo6010-enc.o solo6010-v4l2-enc.o \
		   solo6010-g723.o solo6010-eeprom.o

# For when the kernel isn't compiled with it
ifeq ($(CONFIG_VIDEOBUF_DMA_CONTIG),)
solo6x10-edge-objs += videobuf-dma-contig.o
endif

ifeq ($(CONFIG_VIDEOBUF_DMA_SG),)
solo6x10-edge-objs += videobuf-dma-sg.o
endif

obj-m		:= solo6x10-edge.o

modules modules_install clean: FORCE
	$(MAKE) $(MAKEARGS) -C $(KERNELDIR) M=$(shell pwd) $@

install: modules_install FORCE
	$(if $(KBUILD_VERBOSE:1=),@echo '  BLACKLIST solo6x10')
	$(Q)echo blacklist solo6x10 > /etc/modprobe.d/solo6x10.conf

clean: clean_local

clean_local: FORCE
	rm -f Module.markers modules.order videobuf-dma-contig.c \
	      videobuf-dma-contig.c.in videobuf-dma-sg.c.in


# Workaround for Debian et al
ifeq ($(wildcard $(KERNELSRC)/drivers),)
kerneltar := $(firstword \
		$(wildcard $(patsubst %,/usr/src/linux-source-%.tar.bz2,\
			$(shell uname -r | sed 's@-.*@@;p;s@\.[^.]*$$@@'))))
ifeq ($(kerneltar),)
$(error Missing files on the kernel source directory, and no tarball found)
endif

$(obj)/%.in: $(kerneltar)
	$(if $(KBUILD_VERBOSE:1=),@echo '  EXTRACT' $@)
	$(Q)tar -Oxf $< --wildcards '*/$(@F:.in=)' > $@
else
V4L2SRC = $(wildcard \
	$(KERNELSRC)/drivers/media/video \
	$(KERNELSRC)/drivers/media/v4l2-core)
$(obj)/%.in: $(V4L2SRC)/%
	$(if $(KBUILD_VERBOSE:1=),@echo '  LN' $@)
	$(Q)ln -sf $< $@
endif

$(obj)/videobuf-dma-contig.c $(obj)/videobuf-dma-sg.c: %:%.in
	$(if $(KBUILD_VERBOSE:1=),@echo '  MERGE' $@)
	$(Q)sed '/^MODULE_/d;/^EXPORT_SYMBOL_GPL/d' $< > $@

FORCE:

# For my local crud
-include make.extras
