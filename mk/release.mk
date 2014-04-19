version = $(shell git describe | sed 's/^v//')
tarball = solo6x10-$(version).tar.bz2

upload-dest = lizard.bluecherry.net:/sites/downloads.bluecherrydvr.com/solo6x10/

.PHONY: upload release

upload: release
	scp $(tarball) $(tarball).sig $(upload-dest)

release: $(tarball) $(tarball).sig

%.tar.bz2:
	git archive --prefix=$(@F:.tar.bz2=)/ -o $@ HEAD

%.tar.bz2.sig: %.tar.bz2
	gpg -b -u corp -o $@ $<
