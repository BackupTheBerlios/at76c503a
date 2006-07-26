VERSION = 0.1

CC = gcc
CFLAGS = -g -O2 -Wall

NEWBINFILES = \
	atmel_at76c503-i3861.bin \
	atmel_at76c503-i3863.bin \
	atmel_at76c503-rfmd-acc.bin \
	atmel_at76c503-rfmd.bin \
	atmel_at76c505-rfmd.bin \
	atmel_at76c505-rfmd2958.bin \
	atmel_at76c505a-rfmd2958.bin

OLDBINFILES = \
	atmel_at76c503-rfmd-0.90.2-140.bin \
	atmel_at76c503-rfmd-1.101.0-84.bin \
	atmel_at76c505-rfmd-0.91.0-4.bin \
	atmel_at76c505-rfmd2958-1.101.0-86.bin \
	atmel_at76c505a-rfmd2958-1.102.0-113.bin

BINFILES = $(NEWBINFILES) $(OLDBINFILES)

DISTFILES = README COPYRIGHT

DISTDIR = at76c503a-firmware-$(VERSION)


all: $(BINFILES)

$(BINFILES): gen_fw
	./gen_fw

gen_fw: gen_fw.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f *.bin *.o gen_fw

dist: all
	mkdir $(DISTDIR)
	cp $(NEWBINFILES) $(DISTDIR)
	cp $(DISTFILES) $(DISTDIR)
	for f in $(OLDBINFILES); do \
		ver=`echo $$f | sed 's/^.*-\([^-]*-[^.]*\)\.bin/\1/'`; \
		name=`echo $$f | sed 's/-\([^-]*-[^.]*\)\.bin/.bin/'`; \
		mkdir -p $(DISTDIR)/old/$$ver; \
		cp $$f $(DISTDIR)/old/$$ver/$$name; \
	done
	tar cf - $(DISTDIR) | gzip --best > $(DISTDIR).tar.gz
	rm -rf $(DISTDIR)

headers:
	./fwbin2pkg.sh atmel_at76c503-i3861.bin fw-pkg-i3861.h
	./fwbin2pkg.sh atmel_at76c503-i3863.bin fw-pkg-i3863.h
	./fwbin2pkg.sh atmel_at76c503-rfmd-acc.bin fw-pkg-rfmd-acc.h
	./fwbin2pkg.sh atmel_at76c503-rfmd.bin fw-pkg-rfmd.h
	./fwbin2pkg.sh atmel_at76c505-rfmd.bin fw-pkg-r505.h
	./fwbin2pkg.sh atmel_at76c505-rfmd2958.bin fw-pkg-505-rfmd2958.h
	./fwbin2pkg.sh atmel_at76c505a-rfmd2958.bin fw-pkg-505a-rfmd2958.h

.PHONY: all clean dist headers
