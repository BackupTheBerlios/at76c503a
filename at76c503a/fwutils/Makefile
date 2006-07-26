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
	atmel_at76c503-rfmd-0.100.4-16.bin \

BINFILES = $(NEWBINFILES) $(OLDBINFILES)

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
	mkdir $(DISTDIR)/old
	cp $(OLDBINFILES) $(DISTDIR)/old
	tar cf - $(DISTDIR) | gzip --best > $(DISTDIR).tar.gz
	rm -rf $(DISTDIR)

.PHONY: all clean dist
