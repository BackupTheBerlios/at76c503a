/* -*- linux-c -*- */
/*
 * at76c503-i3861.c:
 *
 * Driver for at76c503-based devices based on the Atmel "Fast-Vnet" reference
 * design using the Intersil 3861 radio chip
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
 * devices which use the Intersil 3861 radio chip.  Almost all of the actual
 * driver is handled by the generic at76c503.c module, this file mostly just
 * deals with the initial probes and downloading the correct firmware to the
 * device before handing it off to at76c503.
 *
 * History:
 *
 * 2003_02_15 0.1: (alex)
 * - created 3861-specific driver file
 *
 * 2003_02_18 0.2: (alex)
 * - Reduced duplicated code and moved as much as possible into at76c503.c
 * - Changed default netdev name to "wlan%d"
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/init.h>

#include "at76c503.h"
#include "usbdfu.h"

/* Include firmware data definition */

#include "fw-i3861.h"

/* Version Information */

#define DRIVER_NAME "at76c503-i3861"
#define DRIVER_AUTHOR \
"Oliver Kurth <oku@masqmail.cx>, Joerg Albert <joerg.albert@gmx.de>, Alex <alex@foogod.com>"
#define DRIVER_DESC "Atmel at76c503 (Intersil 3861) Wireless LAN Driver"

/* USB Device IDs supported by this driver */

#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_503I         0x7603 /* Generic AT76C503/3861 device */

#define VENDOR_ID_LINKSYS             0x066b
#define PRODUCT_ID_LINKSYS_WUSB11_V21 0x2211 /* Linksys WUSB11 v2.1/v2.6 */

#define VENDOR_ID_NETGEAR             0x0864
#define PRODUCT_ID_NETGEAR_MA101A     0x4100 /* Netgear MA 101 Rev. A */

#define VENDOR_ID_TEKRAM              0x0b3b
#define PRODUCT_ID_TEKRAM_U300C       0x1612 /* Tekram U-300C / Allnet ALL0193 */

#define VENDOR_ID_HP                  0x03f0
#define PRODUCT_ID_HP_HN210W          0x011c /* HP HN210W PKW-J7801A */

#define VENDOR_ID_M4Y750              0x0cde /* Unknown Vendor ID! */
#define PRODUCT_ID_M4Y750             0x0001 /* Sitecom/Z-Com/Zyxel M4Y-750 */

#define VENDOR_ID_DYNALINK            0x069a
#define PRODUCT_ID_DYNALINK_WLL013_I  0x0320 /* Dynalink/Askey WLL013 (intersil) */

#define VENDOR_ID_SMC                 0x0d5c
#define PRODUCT_ID_SMC2662W_V1        0xa001 /* EZ connect 11Mpbs
Wireless USB Adapter SMC2662W (v1) */

#define VENDOR_ID_BENQ                0x4a5 /* BenQ (Acer) */
#define PRODUCT_ID_BENQ_AWL_300       0x9000 /* AWL-300 */

/* this adapter contains flash */
#define VENDOR_ID_ADDTRON             0x05dd  /* Addtron */
#define PRODUCT_ID_ADDTRON_AWU120     0xff31 /* AWU-120 */

#define VENDOR_ID_INTEL                        0x8086 /* Intel */
#define PRODUCT_ID_INTEL_AP310         0x0200 /* AP310 AnyPoint II usb */

#define VENDOR_ID_CONCEPTRONIC        0x0d8e
#define PRODUCT_ID_CONCEPTRONIC_C11U  0x7100 /* also Dynalink L11U */

#define VENDOR_ID_ARESCOM		0xd8e
#define PRODUCT_ID_WL_210		0x7110 /* Arescom WL-210, 
						  FCC id 07J-GL2411USB */
static struct usb_device_id dev_table[] = {
	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_503I        ) },
	{ USB_DEVICE(VENDOR_ID_LINKSYS,  PRODUCT_ID_LINKSYS_WUSB11_V21) },
	{ USB_DEVICE(VENDOR_ID_NETGEAR,  PRODUCT_ID_NETGEAR_MA101A    ) },
	{ USB_DEVICE(VENDOR_ID_TEKRAM,   PRODUCT_ID_TEKRAM_U300C      ) },
	{ USB_DEVICE(VENDOR_ID_HP,       PRODUCT_ID_HP_HN210W         ) },
	{ USB_DEVICE(VENDOR_ID_M4Y750,   PRODUCT_ID_M4Y750            ) },
	{ USB_DEVICE(VENDOR_ID_DYNALINK, PRODUCT_ID_DYNALINK_WLL013_I ) },
	{ USB_DEVICE(VENDOR_ID_SMC,      PRODUCT_ID_SMC2662W_V1       ) },
	{ USB_DEVICE(VENDOR_ID_BENQ,     PRODUCT_ID_BENQ_AWL_300      ) },
	{ USB_DEVICE(VENDOR_ID_ADDTRON,  PRODUCT_ID_ADDTRON_AWU120    ) },
	{ USB_DEVICE(VENDOR_ID_INTEL,    PRODUCT_ID_INTEL_AP310       ) },
	{ USB_DEVICE(VENDOR_ID_CONCEPTRONIC,PRODUCT_ID_CONCEPTRONIC_C11U) },
	{ USB_DEVICE(VENDOR_ID_ARESCOM, PRODUCT_ID_WL_210) },
	{ }
};

/* firmware / config variables */

static unsigned char fw_internal[] = FW_I3861_INTERNAL;
static unsigned char fw_external[] = FW_I3861_EXTERNAL;

static int board_type = BOARDTYPE_INTERSIL;

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
	post_download_hook:	at76c503_usbdfu_post
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

	/* HZ became a variable with 2.4.23-preX */
	module_usbdfu.reset_delay=2*HZ;

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
