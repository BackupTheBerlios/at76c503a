/* -*- linux-c -*- */
/*
 * at76c503-rfmd.c:
 *
 * Driver for at76c503-based devices based on the Atmel "Fast-Vnet" reference
 * design using RFMD radio chips
 *
 * Copyright (c) 2002 - 2003 Oliver Kurth <oku@masqmail.cx>
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
 * devices which use radio chips from RF Micro Devices (RFMD).  Almost
 * all of the actual driver is handled by the generic at76c503.c module, this
 * file mostly just deals with the initial probes and downloading the correct
 * firmware to the device before handing it off to at76c503.
 *
 * History:
 *
 * 2003_02_11 0.1: (alex)
 * - split board-specific code off from at76c503.c
 * - reverted to 0.90.2 firmware because 0.100.x is broken for WUSB11
 *
 * 2003_02_18 0.2: (alex)
 * - Reduced duplicated code and moved as much as possible into at76c503.c
 * - Changed default netdev name to "wlan%d"
 */

#include <linux/module.h>
#include <linux/usb.h>

#include "at76c503.h"
#include "usbdfu.h"

/* Include firmware data definition */

#include "fw-rfmd-0.90.2-140.h"

/* Version Information */

#define DRIVER_NAME "at76c503-rfmd"
#define DRIVER_AUTHOR \
"Oliver Kurth <oku@masqmail.cx>, Joerg Albert <joerg.albert@gmx.de>, Alex <alex@foogod.com>"
#define DRIVER_DESC "Atmel at76c503 (RFMD) Wireless LAN Driver"

/* USB Device IDs supported by this driver */

#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_503R         0x7605 /* Generic AT76C503/RFMD device */
#define PRODUCT_ID_W_BUDDIE_WN210     0x4102 /* AirVast W-Buddie WN210 */

#define VENDOR_ID_BELKIN              0x0d5c
#define PRODUCT_ID_BELKIN_F5D6050     0xa002 /* Belkin F5D6050 / SMC 2662W v2 */

#define VENDOR_ID_DYNALINK            0x069a
#define PRODUCT_ID_DYNALINK_WLL013_R  0x0321 /* Dynalink/Askey WLL013 (rfmd) */

#define VENDOR_ID_LINKSYS             0x077b
#define PRODUCT_ID_LINKSYS_WUSB11_V26 0x2219 /* Linksys WUSB11 v2.6 */
#define PRODUCT_ID_NE_NWU11B          0x2227 /* Network Everywhere NWU11B */

#define VENDOR_ID_NETGEAR             0x0864
#define PRODUCT_ID_NETGEAR_MA101B     0x4102 /* Netgear MA 101 Rev. B */

#define VENDOR_ID_ACTIONTEC           0x1668
#define PRODUCT_ID_ACTIONTEC_802UAT1  0x7605 /* Actiontec 802UAT1, HWU01150-01UK */

#define VENDOR_ID_DICK_SMITH_ELECTR   0x1371 /* Dick Smith Electronics */
#define PRODUCT_ID_DSE_XH1153         0x5743 /* XH1153 802.11b USB adapter */

static struct usb_device_id dev_table[] = {
	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_503R        ) },
	{ USB_DEVICE(VENDOR_ID_BELKIN,   PRODUCT_ID_BELKIN_F5D6050    ) },
	{ USB_DEVICE(VENDOR_ID_DYNALINK, PRODUCT_ID_DYNALINK_WLL013_R ) },
	{ USB_DEVICE(VENDOR_ID_LINKSYS,  PRODUCT_ID_LINKSYS_WUSB11_V26) },
	{ USB_DEVICE(VENDOR_ID_LINKSYS,  PRODUCT_ID_NE_NWU11B         ) },
	{ USB_DEVICE(VENDOR_ID_NETGEAR,  PRODUCT_ID_NETGEAR_MA101B    ) },
	{ USB_DEVICE(VENDOR_ID_ACTIONTEC,PRODUCT_ID_ACTIONTEC_802UAT1 ) },
	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_W_BUDDIE_WN210    ) },
	{ USB_DEVICE(VENDOR_ID_DICK_SMITH_ELECTR, PRODUCT_ID_DSE_XH1153) },
	{ }
};

/* firmware / config variables */

static unsigned char fw_internal[] = FW_503RFMD_INTERNAL;
static unsigned char fw_external[] = FW_503RFMD_EXTERNAL;

static int board_type = BOARDTYPE_RFMD;

/*---------------------------------------------------------------------------*/

MODULE_DEVICE_TABLE (usb, dev_table);

/* Module paramaters */

static char netdev_name[IFNAMSIZ+1] = "wlan%d";
MODULE_PARM(netdev_name, "c" __MODULE_STRING(IFNAMSIZ));
MODULE_PARM_DESC(netdev_name,
                 "network device name (default is wlan%d)");

/* local function prototypes */

static void *at76c50x_probe(struct usb_device *dev, unsigned int ifnum,
			     const struct usb_device_id *id);
static void at76c50x_disconnect(struct usb_device *dev, void *ptr);

/* structure for registering this driver with the usb subsystem */

static struct usb_driver module_usb = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,20)
	owner:      THIS_MODULE,
#endif
	name:	    DRIVER_NAME,
	probe:	    at76c50x_probe,
	disconnect: at76c50x_disconnect,
	id_table:   dev_table,
};

/* structure for registering this firmware with the usbdfu subsystem */

static struct usbdfu_info module_usbdfu = {
	name:			DRIVER_NAME,
	id_table:		dev_table,
	fw_buf:			fw_internal,
	fw_buf_len:		sizeof(fw_internal),
	post_download_hook:	at76c503_usbdfu_post,
	reset_delay:            2*HZ
};

/* Module and USB entry points */

static void *at76c50x_probe(struct usb_device *udev, unsigned int ifnum, const struct usb_device_id *id)
{
	if (usbdfu_in_use(udev, ifnum)) {
		/* the device is in DFU mode and usbdfu.c is handling it */
		return NULL;
	}

	return at76c503_do_probe(THIS_MODULE, udev, fw_external, sizeof(fw_external), board_type, netdev_name);
}

static void at76c50x_disconnect(struct usb_device *udev, void *ptr)
{
	info("%s disconnected", ((struct at76c503 *)ptr)->netdev->name);
	at76c503_delete_device(ptr);
}

static int __init mod_init(void)
{
	int result;

	info(DRIVER_DESC " " DRIVER_VERSION);

	/* register with usbdfu so that the firmware will be automatically
	 * downloaded to the device on detection */
	result = usbdfu_register(&module_usbdfu);
	if (result < 0) {
		err("usbdfu_register failed (status %d)", result);
		return -1;
	}

	/* register this driver with the USB subsystem */
	result = usb_register(&module_usb);
	if (result < 0) {
		err("usb_register failed (status %d)", result);
		usbdfu_deregister(&module_usbdfu);
		return -1;
	}

	return 0;
}

static void __exit mod_exit(void)
{
	info(DRIVER_DESC " " DRIVER_VERSION " unloading");
	/* deregister this driver with the USB subsystem */
	usbdfu_deregister(&module_usbdfu);
	usb_deregister(&module_usb);
}

module_init (mod_init);
module_exit (mod_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
