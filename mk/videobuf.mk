v4l2_src := $(wildcard \
	$(KERNELSRC)/drivers/media/video \
	$(KERNELSRC)/drivers/media/v4l2-core)
v4l2_src_test := $(wildcard $(v4l2_src)/videobuf-dma-contig.c)

ifeq ($(v4l2_src_test),)
# Workaround for Debian et al
kerneltar := $(firstword \
		$(wildcard $(patsubst %,/usr/src/linux-source-%.tar.bz2,\
			$(shell uname -r | sed 's@-.*@@;p;s@\.[^.]*$$@@'))))
ifeq ($(kerneltar),)
ifeq ($(basename $(KVERS)),2.6)
ktag = v$(KVERS)
else
ktag = v$(basename $(KVERS))
endif
kurl = https://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/plain
ifeq ($(basename $(ktag))$(filter .0 .1 .2 .3 .4 .5 .6,$(suffix $(ktag))),v3)
v4lpath = drivers/media/v4l2-core
else
v4lpath = drivers/media/video
endif
$(obj)/%.in:
	$(if $(KBUILD_VERBOSE:1=),@echo '  WGET   ' $@)
	$(Q)wget --no-check-cortificate -O $@ "$(kurl)/$(v4lpath)/$(@F:.in=)?id=$(ktag)"
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
