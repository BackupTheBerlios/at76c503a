/* -*- linux-c -*- */
/*
 * Copyright (c) 2002 - 2003 Oliver Kurth
 * Changes Copyright (c) 2004 Joerg Albert <joerg.albert@gmx.de>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation; either version 2 of
 *	the License, or (at your option) any later version.
 *
 * This file is part of the Berlios driver for WLAN USB devices based on the
 * Atmel AT76C503A/505/505A. See at76c503.h for details.
 */

#include <linux/version.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/init.h>
#include <linux/firmware.h>

#ifdef CONFIG_IPAQ_HANDHELD
#include <asm/mach-types.h>
#include <asm/arch/ipaq.h>
#include <asm/arch-pxa/h5400-asic.h>
#endif

#include "at76c503.h"

/* Version Information */

#define DRIVER_NAME "at76fw"
#define DRIVER_AUTHOR \
"Oliver Kurth <oku@masqmail.cx>, Joerg Albert <joerg.albert@gmx.de>, Alex <alex@foogod.com>"
#define DRIVER_DESC "Atmel at76c503a Wireless LAN Driver - firmware loader"

/* This must be in sync with boardtype definitions */
static struct fwentry {
	const char *const fwname;
	const struct firmware *fw;
} firmwares[] = {
	{ "" },
	{ "atmel_at76c503-i3861.bin" },
	{ "atmel_at76c503-i3863.bin" },
	{ "atmel_at76c503-rfmd.bin" },
	{ "atmel_at76c503-rfmd-acc.bin" },
	{ "atmel_at76c505-rfmd.bin" },
	{ "atmel_at76c505-rfmd2958.bin" },
	{ "atmel_at76c505a-rfmd2958.bin" },
	{ "atmel_at76c505amx-rfmd.bin" }
};

/* USB Device IDs supported by this driver */

/* at76c503-i3861 */
#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_503I         0x7603 /* Generic AT76C503/3861 device */

#define VENDOR_ID_LINKSYS_OLD         0x066b
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

#define VENDOR_ID_SMC_OLD             0x0d5c
#define PRODUCT_ID_SMC2662W_V1        0xa001 /* EZ connect 11Mpbs
Wireless USB Adapter SMC2662W (v1) */

#define VENDOR_ID_BENQ                0x04a5 /* BenQ (Acer) */
#define PRODUCT_ID_BENQ_AWL_300       0x9000 /* AWL-300 */

/* this adapter contains flash */
#define VENDOR_ID_ADDTRON             0x05dd  /* Addtron */
#define PRODUCT_ID_ADDTRON_AWU120     0xff31 /* AWU-120 */
/* also Compex WLU11 */

#define VENDOR_ID_INTEL               0x8086 /* Intel */
#define PRODUCT_ID_INTEL_AP310        0x0200 /* AP310 AnyPoint II USB */

#define VENDOR_ID_CONCEPTRONIC        0x0d8e
#define PRODUCT_ID_CONCEPTRONIC_C11U  0x7100 /* also Dynalink L11U */

#define VENDOR_ID_ARESCOM		0xd8e
#define PRODUCT_ID_WL_210		0x7110 /* Arescom WL-210, 
						  FCC id 07J-GL2411USB */
#define VENDOR_ID_IO_DATA		0x04bb
#define PRODUCT_ID_IO_DATA_WN_B11_USB   0x0919 /* IO-DATA WN-B11/USB */

#define VENDOR_ID_BT            0x069a
#define PRODUCT_ID_BT_VOYAGER_1010  0x0821 /* BT Voyager 1010 */


/* at76c503-i3863 */
#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_503_I3863    0x7604 /* Generic AT76C503/3863 device */

#define VENDOR_ID_SAMSUNG             0x055d
#define PRODUCT_ID_SAMSUNG_SWL2100U   0xa000 /* Samsung SWL-2100U */


/* at76c503-rfmd */
#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_503R         0x7605 /* Generic AT76C503/RFMD device */
#define PRODUCT_ID_W_BUDDIE_WN210     0x4102 /* AirVast W-Buddie WN210 */

#define VENDOR_ID_DYNALINK            0x069a
#define PRODUCT_ID_DYNALINK_WLL013_R  0x0321 /* Dynalink/Askey WLL013 (rfmd) */

#define VENDOR_ID_LINKSYS             0x077b
#define PRODUCT_ID_LINKSYS_WUSB11_V26 0x2219 /* Linksys WUSB11 v2.6 */
#define PRODUCT_ID_NE_NWU11B          0x2227 /* Network Everywhere NWU11B */

#define VENDOR_ID_NETGEAR             0x0864
#define PRODUCT_ID_NETGEAR_MA101B     0x4102 /* Netgear MA 101 Rev. B */

#define VENDOR_ID_ACTIONTEC           0x1668
#define PRODUCT_ID_ACTIONTEC_802UAT1  0x7605 /* Actiontec 802UAT1, HWU01150-01UK */

#define VENDOR_ID_DLINK               0x2001 /* D-Link */
#define PRODUCT_ID_DLINK_DWL120       0x3200 /* DWL-120 rev. E */

#define VENDOR_ID_DICK_SMITH_ELECTR   0x1371 /* Dick Smith Electronics */
#define PRODUCT_ID_DSE_XH1153         0x5743 /* XH1153 802.11b USB adapter */
                                             /* also: CNet CNUSB611 (D) */
#define PRODUCT_ID_WL_200U            0x0002 /* WL-200U */

#define VENDOR_ID_BENQ                0x04a5 /* BenQ (Acer) */
#define PRODUCT_ID_BENQ_AWL_400       0x9001 /* BenQ AWL-400 USB stick */

#define VENDOR_ID_3COM                0x506
#define PRODUCT_ID_3COM_3CRSHEW696    0xa01 /* 3COM 3CRSHEW696 */

#define VENDOR_ID_SIEMENS             0x681
#define PRODUCT_ID_SIEMENS_SANTIS_WLL013 0x1b /* Siemens Santis ADSL WLAN 
						 USB adapter WLL 013 */

#define VENDOR_ID_BELKIN_2		0x50d
#define PRODUCT_ID_BELKIN_F5D6050_V2	0x50	/* Belkin F5D6050, version 2 */

#define VENDOR_ID_BLITZ                 0x07b8  
#define PRODUCT_ID_BLITZ_NETWAVE_BWU613 0xb000 /* iBlitzz, BWU613 (not *B or *SB !) */

#define VENDOR_ID_GIGABYTE              0x1044  
#define PRODUCT_ID_GIGABYTE_GN_WLBM101  0x8003 /* Gigabyte GN-WLBM101 */

#define VENDOR_ID_PLANEX                0x2019
#define PRODUCT_ID_PLANEX_GW_US11S      0x3220 /* Planex GW-US11S */

#define VENDOR_ID_COMPAQ                0x049f
#define PRODUCT_ID_IPAQ_INT_WLAN        0x0032 /* internal WLAN adapter in h5[4,5]xx series iPAQs */


/* at76c503-rfmd-acc */
#define VENDOR_ID_BELKIN              0x0d5c
#define PRODUCT_ID_BELKIN_F5D6050     0xa002 /* Belkin F5D6050 / SMC 2662W v2 / SMC 2662W-AR */

#define VENDOR_ID_SMC                 0x083a
#define PRODUCT_ID_SMC_2664W          0x3501


/* at76c505-rfmd */
#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_505R         0x7606 /* Generic AT76C505/RFMD device */


/* at76c505-rfmd2958 */
#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_505R2958     0x7613 /* Generic AT76C505/RFMD device 
					       also OvisLink WL-1130USB */

#define VENDOR_ID_CNET                0x1371
#define PRODUCT_ID_CNET_CNUSB611G     0x0013 /* CNet CNUSB 611G */
#define PRODUCT_ID_FL_WL240U          0x0014 /* Fiberline WL-240U with the
                                                 CNet vendor id */

#define VENDOR_ID_LINKSYS_1915        0x1915 
#define PRODUCT_ID_LINKSYS_WUSB11V28  0x2233 /* Linksys WUSB11 v2.8 */

#define VENDOR_ID_XTERASYS            0x12fd
#define PRODUCT_ID_XTERASYS_XN_2122B  0x1001 /* Xterasys XN-2122B, also
					        IBlitzz BWU613B / BWU613SB */

#define VENDOR_ID_COREGA               0x07aa
#define PRODUCT_ID_COREGA_USB_STICK_11_KK 0x7613 /* Corega WLAN USB Stick 11 (K.K.) */

#define VENDOR_ID_MSI                 0x0db0
#define PRODUCT_ID_MSI_MS6978_WLAN_BOX_PC2PC 0x1020


/* at76c505a-rfmd2958 */
#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_505A         0x7614 /* Generic AT76C505A device */
#define PRODUCT_ID_ATMEL_505AS        0x7617 /* Generic AT76C505AS device */

#define VENDOR_ID_GIGASET             0x1690
#define PRODUCT_ID_GIGASET_11         0x0701


/* at76c505amx-rfmd */
#define VENDOR_ID_ATMEL               0x03eb
#define PRODUCT_ID_ATMEL_505AMX       0x7615 /* Generic AT76C505AMX device */


static struct usb_device_id dev_table[] = {
	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_503I        ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_LINKSYS_OLD, PRODUCT_ID_LINKSYS_WUSB11_V21),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_NETGEAR,  PRODUCT_ID_NETGEAR_MA101A    ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_TEKRAM,   PRODUCT_ID_TEKRAM_U300C      ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_HP,       PRODUCT_ID_HP_HN210W         ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_M4Y750,   PRODUCT_ID_M4Y750            ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_DYNALINK, PRODUCT_ID_DYNALINK_WLL013_I ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_SMC_OLD,  PRODUCT_ID_SMC2662W_V1       ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_BENQ,     PRODUCT_ID_BENQ_AWL_300      ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_ADDTRON,  PRODUCT_ID_ADDTRON_AWU120    ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_INTEL,    PRODUCT_ID_INTEL_AP310       ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_CONCEPTRONIC,PRODUCT_ID_CONCEPTRONIC_C11U),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_ARESCOM, PRODUCT_ID_WL_210),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_IO_DATA, PRODUCT_ID_IO_DATA_WN_B11_USB),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },
	{ USB_DEVICE(VENDOR_ID_BT,       PRODUCT_ID_BT_VOYAGER_1010   ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3861 },

	{ USB_DEVICE(VENDOR_ID_ATMEL,   PRODUCT_ID_ATMEL_503_I3863 ),
	  .driver_info = BOARDTYPE_503_INTERSIL_3863 },
	{ USB_DEVICE(VENDOR_ID_SAMSUNG, PRODUCT_ID_SAMSUNG_SWL2100U),
	  .driver_info = BOARDTYPE_503_INTERSIL_3863 },

	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_503R        ),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_DYNALINK, PRODUCT_ID_DYNALINK_WLL013_R ),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_LINKSYS,  PRODUCT_ID_LINKSYS_WUSB11_V26),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_LINKSYS,  PRODUCT_ID_NE_NWU11B         ),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_NETGEAR,  PRODUCT_ID_NETGEAR_MA101B    ),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_DLINK,    PRODUCT_ID_DLINK_DWL120      ),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_ACTIONTEC,PRODUCT_ID_ACTIONTEC_802UAT1 ),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_W_BUDDIE_WN210    ),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_DICK_SMITH_ELECTR, PRODUCT_ID_DSE_XH1153),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_DICK_SMITH_ELECTR, PRODUCT_ID_WL_200U),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_BENQ,     PRODUCT_ID_BENQ_AWL_400),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_3COM, PRODUCT_ID_3COM_3CRSHEW696),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_SIEMENS,  PRODUCT_ID_SIEMENS_SANTIS_WLL013),
	  .driver_info = BOARDTYPE_503_RFMD },
	{ USB_DEVICE(VENDOR_ID_BELKIN_2, PRODUCT_ID_BELKIN_F5D6050_V2 ),
	  .driver_info = BOARDTYPE_503_RFMD },
        { USB_DEVICE(VENDOR_ID_BLITZ,    PRODUCT_ID_BLITZ_NETWAVE_BWU613 ),
	  .driver_info = BOARDTYPE_503_RFMD },
        { USB_DEVICE(VENDOR_ID_GIGABYTE, PRODUCT_ID_GIGABYTE_GN_WLBM101 ),
	  .driver_info = BOARDTYPE_503_RFMD },
        { USB_DEVICE(VENDOR_ID_PLANEX,   PRODUCT_ID_PLANEX_GW_US11S ),
	  .driver_info = BOARDTYPE_503_RFMD },
        { USB_DEVICE(VENDOR_ID_COMPAQ,   PRODUCT_ID_IPAQ_INT_WLAN),
	  .driver_info = BOARDTYPE_503_RFMD },

	{ USB_DEVICE(VENDOR_ID_SMC, PRODUCT_ID_SMC_2664W),
	  .driver_info = BOARDTYPE_503_RFMD_ACC },
	{ USB_DEVICE(VENDOR_ID_BELKIN,   PRODUCT_ID_BELKIN_F5D6050    ),
	  .driver_info = BOARDTYPE_503_RFMD_ACC },

	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_505R       ),
	  .driver_info = BOARDTYPE_505_RFMD },

	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_505R2958   ),
	  .driver_info = BOARDTYPE_505_RFMD_2958 },
	{ USB_DEVICE(VENDOR_ID_CNET,     PRODUCT_ID_FL_WL240U         ),
	  .driver_info = BOARDTYPE_505_RFMD_2958 },
	{ USB_DEVICE(VENDOR_ID_CNET,     PRODUCT_ID_CNET_CNUSB611G    ),
	  .driver_info = BOARDTYPE_505_RFMD_2958 },
	{ USB_DEVICE(VENDOR_ID_LINKSYS_1915, PRODUCT_ID_LINKSYS_WUSB11V28 ),
	  .driver_info = BOARDTYPE_505_RFMD_2958 },
	{ USB_DEVICE(VENDOR_ID_XTERASYS, PRODUCT_ID_XTERASYS_XN_2122B ),
	  .driver_info = BOARDTYPE_505_RFMD_2958 },
	{ USB_DEVICE(VENDOR_ID_COREGA,   PRODUCT_ID_COREGA_USB_STICK_11_KK ),
	  .driver_info = BOARDTYPE_505_RFMD_2958 },
        { USB_DEVICE(VENDOR_ID_MSI,      PRODUCT_ID_MSI_MS6978_WLAN_BOX_PC2PC),
	  .driver_info = BOARDTYPE_505_RFMD_2958 },

	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_505A       ),
	  .driver_info = BOARDTYPE_505A_RFMD_2958 },
	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_505AS      ),
	  .driver_info = BOARDTYPE_505A_RFMD_2958 },
        { USB_DEVICE(VENDOR_ID_GIGASET,  PRODUCT_ID_GIGASET_11       ),
	  .driver_info = BOARDTYPE_505A_RFMD_2958 },

	{ USB_DEVICE(VENDOR_ID_ATMEL,    PRODUCT_ID_ATMEL_505AMX    ),
	  .driver_info = BOARDTYPE_505AMX_RFMD },

	{ }
};


MODULE_DEVICE_TABLE (usb, dev_table);

/* Module paramaters */

static char* netdev_name = "wlan%d";
module_param(netdev_name, charp,0400);
MODULE_PARM_DESC(netdev_name, "network device name (default is wlan%d)");
static int debug=0;
module_param(debug, bool, 0400);
MODULE_PARM_DESC(debug, "debug on/off");

/* Use our own dbg macro */
#undef dbg
#define dbg(format, arg...) \
	do { \
		if (debug) \
		printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg);\
	} while (0)


/* local function prototypes */
static int at76c50x_probe(struct usb_interface *interface,
			  const struct usb_device_id *id);
static void at76c50x_disconnect(struct usb_interface *interface);


/* structure for registering this driver with the USB subsystem */

static struct usb_driver module_usb = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	.owner = THIS_MODULE,
#endif
	.name = DRIVER_NAME,
	.probe = at76c50x_probe,
	.disconnect = at76c50x_disconnect,
	.id_table = dev_table,
};

/* Module and USB entry points */


static int at76c50x_probe(struct usb_interface *interface,
			  const struct usb_device_id *id)
{
	void *devptr = NULL;
	int retval;

	struct usb_device *udev __attribute__ ((unused));
	int boardtype = (int)id->driver_info;
	const char *const fw_name = firmwares[boardtype].fwname;
	const struct firmware *fw = firmwares[boardtype].fw;
	udev = interface_to_usbdev(interface);

	if (fw == NULL) {
		dbg("downloading firmware %s", fw_name);
		retval = request_firmware(&fw, fw_name, &udev->dev);
		if (retval == 0) {
			dbg("got it.");
		} else {
			err("firmware %s not found.", fw_name);
			/* FIXME: Update URL */
			err("You may need to download the firmware from "
			    "http://www.thekelleys.org.uk/atmel or "
			    "ftp://ftp.berlios.de/pub/at76c503a/firmware/");
			return retval;
		}
	} else
		dbg("re-using previously loaded fw");

	retval = at76c503_do_probe(THIS_MODULE, udev, &module_usb,
				   fw->data, fw->size,
				   boardtype, netdev_name, &devptr);

	return retval;
}

static void at76c50x_disconnect(struct usb_interface *interface)
{
	struct at76c503 *ptr;

	ptr = usb_get_intfdata (interface);
	usb_set_intfdata(interface, NULL);

	info("%s disconnecting", ((struct at76c503 *)ptr)->netdev->name);
	at76c503_delete_device(ptr);
	info(DRIVER_NAME " disconnected");
}

static int __init mod_init(void)
{
	int result;

	info(DRIVER_DESC " " DRIVER_VERSION " loading");

#ifdef CONFIG_IPAQ_HANDHELD
	if (machine_is_h5400()) {
		/* turn WLAN power on */
		/* both needed? */
		SET_H5400_ASIC_GPIO (GPB, RF_POWER_ON, 1);
		SET_H5400_ASIC_GPIO (GPB, WLAN_POWER_ON, 1);
	}
#endif

	/* register this driver with the USB subsystem */
	result = usb_register(&module_usb);
	if (result < 0) {
		err("usb_register failed (status %d)", result);
		return -1;
	}

	return 0;
}

static void __exit mod_exit(void)
{
	int i;

	info(DRIVER_DESC " " DRIVER_VERSION " unloading");
	usb_deregister(&module_usb);
	for (i = 0; i < ARRAY_SIZE(firmwares); i++) {
		if (firmwares[i].fw)
			release_firmware(firmwares[i].fw);
	}

#ifdef CONFIG_IPAQ_HANDHELD
	if (machine_is_h5400()) {
		/* turn WLAN power off */
		SET_H5400_ASIC_GPIO (GPB, RF_POWER_ON, 0);
		SET_H5400_ASIC_GPIO (GPB, WLAN_POWER_ON, 0);
	}
#endif
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
