# Copyright (c) 2002 - 2003 Oliver Kurth
#           (c) 2003 - 2004 Jörg Albert <joerg.albert@gmx.de>
#
#	This program is free software; you can redistribute it and/or
#	modify it under the terms of the GNU General Public License as
#	published by the Free Software Foundation; either version 2 of
#	the License, or (at your option) any later version.
#

# KERNEL_PATH is the path to the Linux kernel build tree.  It is usually
# the same as the kernel tree, except when a separate directory was used
# for the binaries.  By default, we try to compile the modules for the
# currently running kernel.
KERNEL_PATH ?= $(shell readlink -f /lib/modules/`uname -r`/build)

ifeq (,$(KERNEL_PATH))
$(error Kernel tree not found - please set KERNEL_PATH)
endif

VERSION_HEADER = $(KERNEL_PATH)/include/linux/utsrelease.h
ifeq (,$(wildcard $(VERSION_HEADER)))
VERSION_HEADER = $(KERNEL_PATH)/include/linux/version.h
ifeq (,$(wildcard $(VERSION_HEADER)))
$(error Kernel in $(KERNEL_PATH) is not configured)
endif
endif

# Kernel Makefile doesn't always know the exact kernel version, so we
# get it from the kernel headers instead and pass it to make.
KERNELRELEASE = $(shell sed -ne 's/"//g;s/^\#define UTS_RELEASE //p' \
		  $(VERSION_HEADER))


obj-m = at76_usb.o

SRCS = at76_usb.c at76_usb.h compat.h

SPECFILE = at76_usb.spec

DISTFILES = $(SRCS) Makefile README COPYING CHANGELOG kernel_patch.diff

ifndef NO_COMPAT_HEADERS
EXTRA_CFLAGS += -include $(obj)/compat.h
endif

# get the version from at76_usb.h
ifndef M
VERSION = $(shell sed -n 's/^\#define.*DRIVER_VERSION.*"\(.*\)".*$$/\1/p' at76_usb.h)
DISTNAME = at76_usb-$(VERSION)
DISTDIR = $(DISTNAME)
endif

INSTALL_MOD_DIR = kernel/drivers/net/wireless
MODULE_DIR = $(INSTALL_MOD_PATH)/lib/modules/$(KERNELRELEASE)/$(INSTALL_MOD_DIR)
DEPMOD = /sbin/depmod

SPARSE_FLAGS = -Wall -D__CHECK_ENDIAN__

PWD = $(shell pwd)
KBUILD_FLAGS = -C $(KERNEL_PATH) M=$(PWD) KERNELRELEASE=$(KERNELRELEASE)


all: modules

modules:
	$(MAKE) $(KBUILD_FLAGS) modules

check:
	$(MAKE) $(KBUILD_FLAGS) C=2 CF="$(SPARSE_FLAGS)"

install:
	rm -rf $(MODULE_DIR)/at76*
	$(MAKE) $(KBUILD_FLAGS) modules_install \
		INSTALL_MOD_DIR="$(INSTALL_MOD_DIR)"
	$(DEPMOD) -ae

uninstall:
	rm -rf $(MODULE_DIR)/at76*

TAGS:
	rm -f TAGS
	etags --append $(SRCS)

indent:
	for i in $(SRCS); do \
		$(SHELL) ./lindent $$i || exit 1; \
	done

clean:
	rm -f core *.o *~ a.out *.d
	rm -f *.ko *.mod.c .*.cmd
	rm -f *.s *.i
	rm -rf .tmp_versions

ChangeLog: CVS/Entries
	cvs2cl

dist:
	rm -rf $(DISTNAME)
	mkdir $(DISTNAME)
	cp -f $(DISTFILES) $(DISTNAME)
	sed 's/^%define PACKVER.*$$/%define PACKVER $(VERSION)/' \
		$(SPECFILE) >$(DISTNAME)/$(SPECFILE)
	tar zcvf $(DISTNAME).tar.gz $(DISTNAME)
	rm -rf $(DISTNAME)

rmmod:
	-rmmod $(obj-m:%.o=%)	

.PHONY: all modules check install uninstall clean dist rmmod
