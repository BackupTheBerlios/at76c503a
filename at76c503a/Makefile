#$Id: Makefile,v 1.17 2003/12/25 23:28:27 jal2 Exp $

VERSION = 0.12beta1

CC=gcc

KERNEL_VERSION = $(shell uname -r)

KERNEL_SRC = /lib/modules/$(KERNEL_VERSION)/build
KERNEL_HEADERS = $(KERNEL_SRC)/include

MODULES = at76c503.o usbdfu.o at76c503-i3861.o at76c503-rfmd.o at76c503-rfmd-acc.o \
          at76c505-rfmd.o at76c503-i3863.o at76c505-rfmd2958.o

SRCS = at76c503.c usbdfu.c at76c503-rfmd.c at76c505-rfmd.c at76c503-rfmd-acc.c \
       at76c503-i3861.c at76c503-i3863.c at76c505-rfmd2958.c

HDRS = at76c503.h ieee802_11.h usbdfu.h at76c503-fw_skel.c \
	fw-pkg-505-rfmd2958-1.101.0-86.h  fw-pkg-i3863.h  \
	fw-pkg-rfmd-1.101.0-84.h fw-pkg-i3861.h \
	fw-pkg-rfmd-0.90.2-140.h \
	fw-pkg-r505.h fw-pkg-rfmd-acc-1.101.0-84.h

DISTFILES = $(SRCS) $(HDRS) Makefile README COPYING CHANGELOG scripts
TOPDISTFILES = 
DISTNAME = at76c503-$(VERSION)
DISTDIR = ../dist

MODULE_DIR = $(DESTDIR)/lib/modules/$(KERNEL_VERSION)/kernel/drivers/usb/
DEPMOD = /sbin/depmod

# if Rules.make exists in the kernel tree, we assume 2.4 style modules
# if it doesn't assume 2.5 style
OLDMAKE = $(wildcard $(KERNEL_SRC)/Rules.make)

ifeq (,$(OLDMAKE))
# 2.5 style modules, get the kernel makefiles to do the work

SRCDIR = $(shell pwd)
MODVERDIR = $(SRCDIR)/.tmp_versions

obj-m := $(MODULES)

all: modules

modules:
	mkdir -p .tmp_versions
	-cp $(KERNEL_SRC)/.tmp_versions/*.mod $(MODVERDIR)
	$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(SRCDIR) MODVERDIR=$(MODVERDIR) \
		EXTRA_CFLAGS="-g -DDRIVER_VERSION=\\\"v$(VERSION)\\\"" modules

install: all
	mkdir -p $(MODULE_DIR)
	install -m 644 -o 0 -g 0 $(MODULES:%.o=%.ko) $(MODULE_DIR)
	$(DEPMOD) -ae

uninstall:
	for f in $(MODULES:%.o=%.ko); do rm $(MODULE_DIR)/$$f; done

else
# 2.4 style modules

CPPFLAGS = -D__KERNEL__ \
	-DMODULE -DEXPORT_SYMTAB \
	-DDRIVER_VERSION=\"v$(VERSION)\" \
	-I$(KERNEL_HEADERS)
CFLAGS = -O2 -Wall -Wstrict-prototypes -pipe -fno-strict-aliasing \
         -fno-common -Wno-sign-compare -Wno-unused

MODVER = $(shell if cat $(KERNEL_HEADERS)/linux/autoconf.h 2>/dev/null | \
grep -q '^[[:space:]]*\#define[[:space:]]*CONFIG_MODVERSIONS[[:space:]]*1'; \
then echo 1; else echo 0; fi)

ifeq ($(MODVER),1)
MFLAG = -DMODVERSIONS -include $(KERNEL_HEADERS)/linux/modversions.h
endif

CCDEP = $(CC) $(CFLAGS) -M
TAR = tar

all: $(MODULES)

TAGS:
	rm -f TAGS
#	find $(KERNEL_SRC)/ -name '*.[ch]' | xargs etags --append
	etags --append $(SRCS) $(HDRS)

install:
	mkdir -p $(MODULE_DIR)
	for f in $(MODULES); do install -m 644 -o 0 -g 0 $$f $(MODULE_DIR)/$$f; done
	/sbin/depmod -a

uninstall:
	for f in $(MODULES); do rm $(MODULE_DIR)/$$f; done

.PHONY: dist

%.o: %.c
	$(CC) -MD $(CFLAGS) $(CPPFLAGS) $(MFLAG) -c $<

%.s: %.c
	$(CC) -MD $(CFLAGS) $(CPPFLAGS) -S $<

%.i: %.c
	$(CC) -MD $(CPPFLAGS) -E $< -o $@

-include $(SRCS:%.c=%.d)

endif

clean:
	rm -f core *.o *~ a.out *.d
	rm -f *.ko *.mod.c .*.cmd
	rm -f *.s *.i
	rm -rf .tmp_versions

dist:
	[ -d $(DISTNAME) ] && rm -rf $(DISTNAME) || true
	mkdir $(DISTNAME)
	cp -aR $(DISTFILES) $(DISTNAME)
	tar zcvf $(DISTNAME).tar.gz $(DISTNAME)
	rm -rf $(DISTNAME)

