.PHONY: check

check: checkpatch.report

checkpatch.report:
	-$(KERNELSRC)/scripts/checkpatch.pl -f $(filter-out compat.h videobuf-dma-%,$(wildcard *.c *.h)) > $@
	-$(EDITOR) $@
