.PHONY: update-linux build-%

build-kvers = $(shell echo '$(@:build-%=%)' | sed 's/-.*//')

build-%: autobuild/linux-%
	make KERNELSRC=$< KERNELDIR=$< KVERS=$(build-kvers)

k-prefix = git://git.kernel.org/pub/scm/linux/kernel/git

autobuild/linux.git:
	git clone --bare $(k-prefix)/torvalds/linux.git $@
	GIT_DIR=$@ git remote add stable $(k-prefix)/stable/linux-stable.git

update-linux: autobuild/linux.git
	GIT_DIR=autobuild/linux.git git fetch --all

autobuild/linux-%: autobuild/linux.git
	make -C autobuild -f ../mk/kernel-checkout.mk $(notdir $@)
