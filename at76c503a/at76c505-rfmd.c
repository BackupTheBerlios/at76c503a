/* -*- linux-c -*- */
/*
 * $Id: at76c505-rfmd.c,v 1.10 2004/01/17 11:14:11 jal2 Exp $
 *
 * Driver for at76c503-based devices based on the Atmel "Fast-Vnet" reference
 * design using at76c505 with RFMD radio chips
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
 * This driver contains code specific to Atmel AT76C505 (USB wireless 802.11)
 * devices which use radio chips from RF Micro Devices (RFMD).  Almost
 * all of the actual driver is handled by the generic at76c503.c module, this
 * file just registers for the USB ids and passes the correct firmware to
 * at76c503.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/init.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,23)
#include <linux/firmware.h>
#endif

#include "at76c503.h"

/* Include firmware data definition: a dummy or a statically compiled-in fw */
#ifdef CONFIG_AT76C503_FIRMWARE_DOWNLOAD
# include "fw-empty.h"
#else
# include "fw-pkg-r505.h"
#endif

/* Version Information */

#define DRIVER_NAME "at76c503-r505"
#define DRIVER_AUTHOR \
"Oliver Kurth <oku@masqmail.cx>, Joerg Albert <joerg.albert@gmx.de>, Alex <alex@foogod.com>"
#define DRIVER_DESC "Atmel at76c505 (RFMD radio) Wireless LAN Driver"

#define BOARDTYPE BOARDTYPE_505_RFMD

/* firmware name to load if above include file contains empty fw only */
#define FW_NAME DRIVER_NAME "-fw"

/* USB Device IDs supported by this driver */

#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_505R         0x7606 /* Generic AT76C505/RFMD device */

static struct usb_device_id dev_table[] = {
	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_505R       ) },
	{ }
};

/* jal: not really good style to include a .c file, but all but the above
   is constant in the at76c503-*.c files ... */
#include "at76c503-fw_skel.c"
