.PHONY: $(MAKECMDGOALS) none
none:
$(MAKECMDGOALS): %:%/.config

arch = x86_64
platform = $(arch)

.PRECIOUS: %/Makefile
%/Makefile:
	git clone -s linux.git \
		-b v$(patsubst %.0,%,$(@:linux-%/Makefile=%)) \
		$(dir $@)

%/.config: %/Makefile
	$(MAKE) -C $(dir $@) mrproper $(platform)_defconfig prepare
