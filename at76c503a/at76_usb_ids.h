/*
 * USB IDs for the at76c503/at76c505 USB driver
 *
 * Copyright (c) 2007 Guido Guenther <agx@sigxcpu.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef _AT76_USB_IDS_H
#define _AT76_USB_IDS_H

#define VID_3COM		0x0506
#define VID_ACTIONTEC		0x1668
#define VID_ADDTRON		0x05dd
#define VID_ARESCOM		0x0d8e
#define VID_ATMEL		0x03eb
#define VID_BELKIN		0x0d5c
#define VID_BELKIN_2		0x050d
#define VID_BENQ		0x04a5
#define VID_BLITZ		0x07b8
#define VID_BT			0x069a
#define VID_CNET		0x1371
#define VID_COMPAQ		0x049f
#define VID_CONCEPTRONIC	0x0d8e
#define VID_COREGA		0x07aa
#define VID_DICK_SMITH_ELECTR	0x1371	/* Dick Smith Electronics */
#define VID_DLINK		0x2001
#define VID_DYNALINK		0x069a
#define VID_GIGABYTE		0x1044
#define VID_GIGASET		0x1690
#define VID_HP			0x03f0
#define VID_INTEL		0x8086
#define VID_IO_DATA		0x04bb
#define VID_LINKSYS		0x077b
#define VID_LINKSYS_1915	0x1915
#define VID_LINKSYS_OLD		0x066b
#define VID_MSI			0x0db0
#define VID_M4Y750		0x0cde	/* Unknown Vendor ID */
#define VID_NETGEAR		0x0864
#define VID_SAMSUNG		0x055d
#define VID_SIEMENS		0x0681
#define VID_SMC			0x083a
#define VID_SMC_OLD		0x0d5c
#define VID_PLANEX		0x2019
#define VID_TEKRAM		0x0b3b
#define VID_XTERASYS		0x12fd

#define PID_ATMEL_503I		0x7603	/* Generic AT76C503/3861 device */
#define PID_LINKSYS_WUSB11_V21	0x2211	/* Linksys WUSB11 v2.1/v2.6 */
#define PID_NETGEAR_MA101A	0x4100	/* Netgear MA 101 Rev. A */
#define PID_TEKRAM_U300C	0x1612	/* Tekram U-300C / Allnet ALL0193 */
#define PID_HP_HN210W		0x011c	/* HP HN210W PKW-J7801A */
#define PID_M4Y750		0x0001	/* Sitecom/Z-Com/Zyxel M4Y-750 */
#define PID_DYNALINK_WLL013_I	0x0320	/* Dynalink/Askey WLL013 (intersil) */
#define PID_SMC2662W_V1		0xa001	/* EZ connect 11Mpbs
					   Wireless USB Adapter SMC2662W (v1) */
#define PID_BENQ_AWL_300	0x9000	/* AWL-300 */
#define PID_ADDTRON_AWU120	0xff31	/* AWU-120, Compex WLU11 */
#define PID_INTEL_AP310		0x0200	/* AP310 AnyPoint II USB */
#define PID_CONCEPTRONIC_C11U	0x7100	/* also Dynalink L11U */
#define PID_WL_210		0x7110	/* Arescom WL-210,
					   FCC id 07J-GL2411USB */
#define PID_IO_DATA_WN_B11_USB	0x0919	/* IO-DATA WN-B11/USB */
#define PID_BT_VOYAGER_1010	0x0821	/* BT Voyager 1010 */
#define PID_ATMEL_503_I3863	0x7604	/* Generic AT76C503/3863 device */
#define PID_SAMSUNG_SWL2100U	0xa000	/* Samsung SWL-2100U */
#define PID_ATMEL_503R		0x7605	/* Generic AT76C503/RFMD device */
#define PID_W_BUDDIE_WN210	0x4102	/* AirVast W-Buddie WN210 */
#define PID_DYNALINK_WLL013_R	0x0321	/* Dynalink/Askey WLL013 (rfmd) */
#define PID_LINKSYS_WUSB11_V26	0x2219	/* Linksys WUSB11 v2.6 */
#define PID_NE_NWU11B		0x2227	/* Network Everywhere NWU11B */
#define PID_NETGEAR_MA101B	0x4102	/* Netgear MA 101 Rev. B */
#define PID_ACTIONTEC_802UAT1	0x7605	/* Actiontec 802UAT1, HWU01150-01UK */
#define PID_DLINK_DWL120	0x3200	/* DWL-120 rev. E */
#define PID_DSE_XH1153		0x5743	/* XH1153 802.11b USB adapter */
#define PID_WL_200U		0x0002	/* WL-200U */
#define PID_BENQ_AWL_400	0x9001	/* BenQ AWL-400 USB stick */
#define PID_3COM_3CRSHEW696	0x0a01	/* 3COM 3CRSHEW696 */
#define PID_SIEMENS_SANTIS_WLL013 0x001b	/* Siemens Santis ADSL WLAN
						   USB adapter WLL 013 */
#define PID_BELKIN_F5D6050_V2	0x0050	/* Belkin F5D6050, version 2 */
#define PID_BLITZ_NETWAVE_BWU613 0xb000	/* iBlitzz, BWU613 (not *B or *SB) */
#define PID_GIGABYTE_GN_WLBM101	0x8003	/* Gigabyte GN-WLBM101 */
#define PID_PLANEX_GW_US11S	0x3220	/* Planex GW-US11S */
#define PID_IPAQ_INT_WLAN	0x0032	/* internal WLAN adapter in h5[4,5]xx
					   series iPAQs */
#define PID_BELKIN_F5D6050	0xa002	/* Belkin F5D6050, SMC 2662W v2,
					   SMC 2662W-AR */
#define PID_SMC_2664W		0x3501
#define PID_ATMEL_505R		0x7606	/* Generic AT76C505/RFMD */
#define PID_ATMEL_505R2958	0x7613	/* Generic AT76C505/RFMD,
					   OvisLink WL-1130USB */
#define PID_CNET_CNUSB611G	0x0013	/* CNet CNUSB 611G */
#define PID_FL_WL240U		0x0014	/* Fiberline WL-240U (CNet vendor id) */
#define PID_LINKSYS_WUSB11V28	0x2233	/* Linksys WUSB11 v2.8 */
#define PID_XTERASYS_XN_2122B	0x1001	/* Xterasys XN-2122B,
					   IBlitzz BWU613B/BWU613SB */
#define PID_COREGA_USB_STICK_11_KK 0x7613	/* Corega WLAN USB Stick 11 */
#define PID_MSI_MS6978_WLAN_BOX_PC2PC 0x1020
#define PID_ATMEL_505A		0x7614	/* Generic AT76C505A device */
#define PID_ATMEL_505AS		0x7617	/* Generic AT76C505AS device */
#define PID_GIGASET_11		0x0701
#define PID_ATMEL_505AMX	0x7615	/* Generic AT76C505AMX device */

#define BOARDTYPE_503_INTERSIL_3861	1
#define BOARDTYPE_503_INTERSIL_3863	2
#define BOARDTYPE_503_RFMD		3
#define BOARDTYPE_503_RFMD_ACC		4
#define BOARDTYPE_505_RFMD		5
#define BOARDTYPE_505_RFMD_2958		6
#define BOARDTYPE_505A_RFMD_2958	7
#define BOARDTYPE_505AMX_RFMD		8

#endif				/* _AT76_USB_IDS_H */
