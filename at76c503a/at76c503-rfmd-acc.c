/* -*- linux-c -*- */
/*
 * $Id: at76c503-rfmd-acc.c,v 1.8 2004/01/17 11:14:11 jal2 Exp $
 *
 * Driver for at76c503-based devices based on the Atmel "Fast-Vnet" reference
 * design using RFMD radio chips in the Accton OEM layout.
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
 * devices which use radio chips from RF Micro Devices (RFMD) with Accton OEM layout. 
 * Almost all of the actual driver is handled by the generic at76c503.c module, this
 * file just registers for the USB ids and passes the correct firmware to
 * at76c503.
 *
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
# include "fw-pkg-rfmd-acc-1.101.0-84.h"
#endif

/* Version Information */

#define DRIVER_NAME "at76c503-rfmd-acc"
#define DRIVER_AUTHOR \
"Oliver Kurth <oku@masqmail.cx>, Joerg Albert <joerg.albert@gmx.de>, Alex <alex@foogod.com>"
#define DRIVER_DESC "Atmel at76c503 (RFMD/Accton) Wireless LAN Driver"

/* firmware name to load if above include file contains empty fw only */
#define FW_NAME DRIVER_NAME "-fw"

#define BOARDTYPE BOARDTYPE_503_RFMD_ACC

/* USB Device IDs supported by this driver */

#define VENDOR_ID_BELKIN              0x0d5c
#define PRODUCT_ID_BELKIN_F5D6050     0xa002 /* Belkin F5D6050 / SMC 2662W v2 / SMC 2662W-AR */

#define VENDOR_ID_SMC                 0x083a
#define PRODUCT_ID_SMC_2664W          0x3501

static struct usb_device_id dev_table[] = {
	{ USB_DEVICE(VENDOR_ID_SMC, PRODUCT_ID_SMC_2664W) },
	{ USB_DEVICE(VENDOR_ID_BELKIN,   PRODUCT_ID_BELKIN_F5D6050    ) },
	{ }
};
/*---------------------------------------------------------------------------*/

/* jal: not really good style to include a .c file, but all but the above
   is constant in the at76c503-*.c files ... */
#include "at76c503-fw_skel.c"
