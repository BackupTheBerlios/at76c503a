/* -*- linux-c -*- */
/*
 * $Id: at76c503-i3863.c,v 1.7 2003/12/27 00:03:34 jal2 Exp $
 *
 * Driver for at76c503-based devices based on the Atmel "Fast-Vnet" reference
 * design using a Intersil 3863 radio chip
 *
 * Copyright (c) 2002 - 2003 Oliver Kurth <oku@masqmail.cx>
 * Changes Copyright (c) 2003 Joerg Albert <joerg.albert@gmx.de>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation; either version 2 of
 *	the License, or (at your option) any later version.
 *
 *
 * This driver is derived from usb-skeleton.c
 *
 * This driver contains code specific to Atmel AT76C503 (USB wireless 802.11)
 * devices which use the Intersil 3863 radio chip.  Almost
 * all of the actual driver is handled by the generic at76c503.c module, this
 * file just registers for the USB ids and passes the correct firmware to
 * at76c503.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/init.h>
#include <linux/firmware.h>

#include "at76c503.h"

/* Include firmware data definition: a dummy or a statically compiled-in fw */
#ifdef CONFIG_AT76C503_FIRMWARE_DOWNLOAD
# include "fw-empty.h"
#else
# include "fw-pkg-i3863.h"
#endif

/* Version Information */

#define DRIVER_NAME "at76c503-i3863"
#define DRIVER_AUTHOR \
"Oliver Kurth <oku@masqmail.cx>, Joerg Albert <joerg.albert@gmx.de>, Alex <alex@foogod.com>"
#define DRIVER_DESC "Atmel at76c503 (i3863) Wireless LAN Driver"

#define BOARDTYPE BOARDTYPE_503_INTERSIL_3863

/* firmware name to load if above include file contains empty fw only */
#define FW_NAME DRIVER_NAME "-fw"

#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_503_I3863    0x7604 /* Generic AT76C503/3863 device */

#define VENDOR_ID_SAMSUNG             0x055d
#define PRODUCT_ID_SAMSUNG_SWL2100U   0xa000 /* Samsung SWL-2100U */

static struct usb_device_id dev_table[] = {
	{ USB_DEVICE(VENDOR_ID_ATMEL,   PRODUCT_ID_ATMEL_503_I3863 ) },
	{ USB_DEVICE(VENDOR_ID_SAMSUNG, PRODUCT_ID_SAMSUNG_SWL2100U) },
	{ }
};


/* jal: not really good style to include a .c file, but all but the above
   is constant in the at76c503-*.c files ... */
#include "at76c503-fw_skel.c"
