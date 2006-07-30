VERSION = 0.1

CC = gcc
CFLAGS = -g -O2 -Wall

BINFILES = \
	atmel_at76c503-i3861-0.90.0-44.bin \
	atmel_at76c503-i3863-0.90.0-44.bin \
	atmel_at76c503-rfmd-0.90.2-140.bin \
	atmel_at76c503-rfmd-1.101.0-84.bin \
	atmel_at76c503-rfmd-1.103.0-175.bin \
	atmel_at76c503-rfmd-acc-1.101.0-84.bin \
	atmel_at76c505-rfmd-0.91.0-4.bin \
	atmel_at76c505-rfmd-1.103.0-175.bin \
	atmel_at76c505-rfmd2958-1.101.0-86.bin \
	atmel_at76c505-rfmd2958-1.103.0-175.bin \
	atmel_at76c505a-rfmd2958-1.102.0-113.bin \
	atmel_at76c505a-rfmd2958-1.103.0-175.bin \
	atmel_at76c505amx-1.103.0-175.bin

GOODBINFILES = \
	atmel_at76c503-i3861-0.90.0-44.bin \
	atmel_at76c503-i3863-0.90.0-44.bin \
	atmel_at76c503-rfmd-1.101.0-84.bin \
	atmel_at76c503-rfmd-acc-1.101.0-84.bin \
	atmel_at76c505-rfmd-0.91.0-4.bin \
	atmel_at76c505-rfmd2958-1.101.0-86.bin \
	atmel_at76c505a-rfmd2958-1.102.0-113.bin \

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
	rm -rf $(DISTDIR)
	mkdir $(DISTDIR)
	cp $(DISTFILES) $(DISTDIR)
	mkdir $(DISTDIR)/all
	cp $(BINFILES) $(DISTDIR)/all
	mkdir $(DISTDIR)/good
	for f in $(GOODBINFILES); do \
		ver=`echo $$f | sed 's/^.*-\([^-]*-[^.]*\)\.bin/\1/'`; \
		name=`echo $$f | sed 's/-\([^-]*-[^.]*\)\.bin/.bin/'`; \
		ln -s ../all/$$f $(DISTDIR)/good/$$name; \
	done
	tar cf - $(DISTDIR) | gzip --best > $(DISTDIR).tar.gz
	rm -rf $(DISTDIR)

.PHONY: all clean dist headers
