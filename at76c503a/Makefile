#$Id: Makefile,v 1.49 2006/07/15 03:12:32 proski Exp $
#
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


FW_DL=

# compile for firmware download from user space
# if no firmware header files are available
ifneq ($(wildcard fw-pkg*.h),)
FW_DL += -DCOMPILE_FIRMWARE_INTO_DRIVER
endif

obj-m = at76c503-i3861.o at76c503-rfmd.o at76c503-rfmd-acc.o \
	at76c505-rfmd.o at76c503-i3863.o at76c505-rfmd2958.o \
	at76c505a-rfmd2958.o at76c503.o at76_usbdfu.o 

SRCS = at76c503.c at76_usbdfu.c at76c503-rfmd.c at76c505-rfmd.c at76c503-rfmd-acc.c \
       at76c503-i3861.c at76c503-i3863.c at76c505-rfmd2958.c at76c505a-rfmd2958.c

HDRS = at76c503.h at76_ieee802_11.h at76_usbdfu.h at76c503-fw_skel.c
FW_HDRS = fw-pkg-505-rfmd2958-1.101.0-86.h fw-pkg-i3863.h \
	fw-pkg-rfmd-1.101.0-84.h fw-pkg-i3861.h \
	fw-pkg-rfmd-0.90.2-140.h \
	fw-pkg-r505.h fw-pkg-rfmd-acc-1.101.0-84.h \
	fw-pkg-505a-rfmd2958-1.102.0-113.h

SCRIPTS = fwbin2h fwbin2pkg.sh fwconvert fwversion

DISTFILES = $(SRCS) $(HDRS) $(FW_HDRS) Makefile README COPYING \
            CHANGELOG gen_fw.c kernel_patch.sh Makefile.k26

# get the version from at76c503.h
ifndef M
VERSION = $(shell sed -n 's/^\#define.*DRIVER_VERSION.*"\(.*\)".*$$/\1/p' at76c503.h)
DISTNAME = at76c503-$(VERSION)
DISTDIR = $(DISTNAME)
endif

INSTALL_MOD_DIR = kernel/drivers/net/wireless/at76c503
MODULE_DIR = $(DESTDIR)/lib/modules/$(KERNELRELEASE)/$(INSTALL_MOD_DIR)
DEPMOD = /sbin/depmod

SPARSE_FLAGS = -Wbitwise -Wcast-to-as -Wcontext -Wdefault-bitfield-sign \
	-Wtransparent-union -Wptr-subtraction-blows -Wundef -Wdecl \
	-Wone-bit-signed-bitfield -Wtypesign -D__CHECK_ENDIAN__

PWD = $(shell pwd)
KBUILD_FLAGS = -C $(KERNEL_PATH) M=$(PWD) KERNELRELEASE=$(KERNELRELEASE) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS) $(FW_DL)"


all: modules

modules:
	$(MAKE) $(KBUILD_FLAGS) modules

check:
	$(MAKE) $(KBUILD_FLAGS) C=2 CF="$(SPARSE_FLAGS)"

install:
	$(MAKE) $(KBUILD_FLAGS) modules_install \
		INSTALL_MOD_DIR="$(INSTALL_MOD_DIR)"
	$(DEPMOD) -ae

uninstall:
	for f in $(obj-m:%.o=%.ko); do rm -f $(MODULE_DIR)/$$f; done

.PHONY: all dist kernel_patch rmmod install modules

TAGS:
	rm -f TAGS
	etags --append $(SRCS) $(HDRS)

clean:
	rm -f core *.o *~ a.out *.d
	rm -f *.ko *.mod.c .*.cmd
	rm -f *.s *.i
	rm -rf .tmp_versions

dist:
	[ -d $(DISTNAME) ] && rm -rf $(DISTNAME) || true
	mkdir $(DISTNAME)
	cp -f $(DISTFILES) $(DISTNAME)
	mkdir $(DISTNAME)/scripts
	for f in $(SCRIPTS); do \
		cp -f scripts/$$f $(DISTNAME)/scripts/ || exit 1; \
	done
	tar zcvf $(DISTNAME).tar.gz $(DISTNAME)
	rm -f $(DISTNAME)/fw-pkg-*.h
	mv $(DISTNAME) $(DISTNAME)-fwdl
	tar zcvf $(DISTNAME)-fwdl.tar.gz $(DISTNAME)-fwdl
	rm -rf $(DISTNAME)-fwdl

kernel_patch:
	 DRV_SRC="$(SRCS)" DRV_HDR="$(HDRS)" ./kernel_patch.sh $(KERNEL_PATH)

rmmod:
	-rmmod $(obj-m:%.o=%)	
