VERSION = 0.8

CC=gcc

KERNEL_VERSION = $(shell uname -r)

KERNEL_SRC = /lib/modules/$(KERNEL_VERSION)/build
KERNEL_HEADERS = $(KERNEL_SRC)/include

MODULES = at76c503.o usbdfu.o

SRCS = at76c503.c usbdfu.c at76c503fw.c rom2h.c
HDRS = internalr.h externalr.h at76c503.h ieee802_11.h usbdfu.h
MODULE_DIR = $(DESTDIR)/lib/modules/$(KERNEL_VERSION)/kernel/drivers/usb/
CPPFLAGS = -D__KERNEL__ \
	-DMODULE -DEXPORT_SYMTAB \
	-DDRIVER_VERSION=\"v$(VERSION)\" \
	-I$(KERNEL_HEADERS)
CFLAGS = -O2 -Wall -Wstrict-prototypes -pipe

MODVER = $(shell if cat $(KERNEL_HEADERS)/linux/autoconf.h 2>/dev/null | \
grep -q '^[[:space:]]*\#define[[:space:]]*CONFIG_MODVERSIONS[[:space:]]*1'; \
then echo 1; else echo 0; fi)

ifeq ($(MODVER),1)
MFLAG = -DMODVERSIONS -include $(KERNEL_HEADERS)/linux/modversions.h
endif

CCDEP = $(CC) $(CFLAGS) -M
TAR = tar

DISTFILES = $(SRCS) $(HDRS) Makefile README COPYING
TOPDISTFILES = 
DISTNAME = at76c503-$(VERSION)
DISTDIR = ../dist

all: $(MODULES)

at76c503fw: at76c503fw.c
	$(CC) $(CFLAGS) -o $@ $< -lusb

rom2h: rom2h.c
	$(CC) $(CFLAGS) -o $@ $<

TAGS:
	rm -f TAGS
	find $(KERNEL_SRC)/ -name '*.[ch]' | xargs etags --append
	etags --append $(SRCS) $(HDRS)

install: all
	mkdir -p $(MODULE_DIR)
	for f in $(MODULES); do install -m 644 -o 0 -g 0 $$f $(MODULE_DIR)/$$f; done
	/sbin/depmod -a

uninstall:
	for f in $(MODULES); do rm $(MODULE_DIR)/$$f; done

clean:
	rm -f at76c503fw rom2h
	rm -f core *.o *~ a.out *.d
	rm -f *.s *.i

dist:
	[ -d $(DISTNAME) ] && rm -rf $(DISTNAME) || true
	mkdir $(DISTNAME)
	cp -aR $(DISTFILES) $(DISTNAME)
	tar zcvf $(DISTNAME).tar.gz $(DISTNAME)
	rm -rf $(DISTNAME)

.PHONY: dist

%.o: %.c
	$(CC) -MD $(CFLAGS) $(CPPFLAGS) $(MFLAG) -c $<

%.s: %.c
	$(CC) -MD $(CFLAGS) $(CPPFLAGS) -S $<

%.i: %.c
	$(CC) -MD $(CPPFLAGS) -E $< -o $@

-include $(SRCS:%.c=%.d)
