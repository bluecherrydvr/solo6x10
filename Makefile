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

FORCE:;

-include mk/release.mk
-include mk/checks.mk
-include mk/autobuild.mk

# For my local crud
-include make.extras


### Videobuf stuff
# For when the kernel isn't compiled with videobuf-dma-*
ifneq ($(CONFIG_VIDEOBUF_DMA_CONTIG),y)
solo6x10-edge-y$(CONFIG_VIDEOBUF_DMA_CONTIG)	+= videobuf-dma-contig.o
endif
ifneq ($(CONFIG_VIDEOBUF_DMA_SG),y)
solo6x10-edge-y$(CONFIG_VIDEOBUF_DMA_SG)	+= videobuf-dma-sg.o
endif

ifneq ($(CONFIG_VIDEOBUF_DMA_CONTIG)$(CONFIG_VIDEOBUF_DMA_SG),yy)
v4l2_src := $(wildcard \
	$(KERNELSRC)/drivers/media/video \
	$(KERNELSRC)/drivers/media/v4l2-core)
v4l2_src_test := $(wildcard $(v4l2_src)/videobuf-dma-contig.c)

ifeq ($(v4l2_src_test),)
# Workaround for Debian et al
kerneltar := $(firstword \
		$(wildcard $(patsubst %,/usr/src/linux-source-%.tar.bz2,\
			$(shell uname -r | sed 's@-.*@@;p;s@\.[^.]*$$@@'))))

kupver=$(shell echo $(KVERS) | sed 's/-.*//')
ifeq ($(kerneltar),)
ifeq ($(basename $(kupver)),2.6)
ktag = v$(kupver)
else
ktag = v$(basename $(kupver))
endif
kurl = https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/plain
ifeq ($(basename $(ktag))$(filter .0 .1 .2 .3 .4 .5 .6,$(suffix $(ktag))),v3)
v4lpath = drivers/media/v4l2-core
else
v4lpath = drivers/media/video
endif
$(obj)/%.in:
	$(if $(KBUILD_VERBOSE:1=),@echo '  WGET   ' $@)
	$(Q)wget --no-check-certificate -O $@ "$(kurl)/$(v4lpath)/$(@F:.in=)?id=$(ktag)"
else
$(obj)/%.in: $(kerneltar)
	$(if $(KBUILD_VERBOSE:1=),@echo '  EXTRACT' $@)
	$(Q)tar -Oxf $< --wildcards '*/$(@F:.in=)' > $@
endif
else

$(obj)/%.in: $(v4l2_src)/%
	$(if $(KBUILD_VERBOSE:1=),@echo '  LN     ' $@)
	$(Q)ln -sf $< $@
endif

$(obj)/videobuf-dma-contig.c $(obj)/videobuf-dma-sg.c: %:%.in
	$(if $(KBUILD_VERBOSE:1=),@echo '  MERGE  ' $@)
	$(Q)sed '/^MODULE_/d;/^EXPORT_SYMBOL_GPL/d' $< > $@
endif
