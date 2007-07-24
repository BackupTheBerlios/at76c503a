/*
 * at76c503/at76c505 USB driver
 *
 * Copyright (c) 2002 - 2003 Oliver Kurth
 * Copyright (c) 2004 Joerg Albert <joerg.albert@gmx.de>
 * Copyright (c) 2004 Nick Jones
 * Copyright (c) 2004 Balint Seeber <n0_5p4m_p13453@hotmail.com>
 * Copyright (c) 2007 Guido Guenther <agx@sigxcpu.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This file is part of the Berlios driver for WLAN USB devices based on the
 * Atmel AT76C503A/505/505A.
 *
 * Some iw_handler code was taken from airo.c, (C) 1999 Benjamin Reed
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/usb.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/wireless.h>
#include <net/iw_handler.h>
#include <net/ieee80211_radiotap.h>
#include <linux/firmware.h>
#include <linux/leds.h>

#include "at76_usb.h"

/* Version information */
#define DRIVER_NAME "at76_usb"
#define DRIVER_DESC "Atmel at76x USB Wireless LAN Driver"

static int at76_debug = DBG_DEFAULTS;

/* Protect against concurrent firmware loading and parsing */
static struct mutex fw_mutex;

static struct fwentry firmwares[] = {
	[0] = {""},
	[BOARD_503_ISL3861] = {"atmel_at76c503-i3861.bin"},
	[BOARD_503_ISL3863] = {"atmel_at76c503-i3863.bin"},
	[BOARD_503] = {"atmel_at76c503-rfmd.bin"},
	[BOARD_503_ACC] = {"atmel_at76c503-rfmd-acc.bin"},
	[BOARD_505] = {"atmel_at76c505-rfmd.bin"},
	[BOARD_505_2958] = {"atmel_at76c505-rfmd2958.bin"},
	[BOARD_505A_2958] = {"atmel_at76c505a-rfmd2958.bin"},
	[BOARD_505AMX] = {"atmel_at76c505amx-rfmd.bin"},
};

static struct usb_device_id dev_table[] = {
	/*
	 * at76c503-i3861
	 */
	/* Generic AT76C503/3861 device */
	{USB_DEVICE(VID_ATMEL, 0x7603), .driver_info = BOARD_503_ISL3861},
	/* Linksys WUSB11 v2.1/v2.6 */
	{USB_DEVICE(VID_LINKSYS_OLD, 0x2211), .driver_info = BOARD_503_ISL3861},
	/* Netgear MA 101 Rev. A */
	{USB_DEVICE(VID_NETGEAR, 0x4100), .driver_info = BOARD_503_ISL3861},
	/* Tekram U-300C / Allnet ALL0193 */
	{USB_DEVICE(VID_TEKRAM, 0x1612), .driver_info = BOARD_503_ISL3861},
	/* HP HN210W PKW-J7801A */
	{USB_DEVICE(VID_HP, 0x011c), .driver_info = BOARD_503_ISL3861},
	/* Sitecom/Z-Com/Zyxel M4Y-750 */
	{USB_DEVICE(VID_ZCOM, 0x0001), .driver_info = BOARD_503_ISL3861},
	/* Dynalink/Askey WLL013 (intersil) */
	{USB_DEVICE(VID_ASKEY, 0x0320), .driver_info = BOARD_503_ISL3861},
	/* EZ connect 11Mpbs Wireless USB Adapter SMC2662W (v1) */
	{USB_DEVICE(VID_SMC_OLD, 0xa001), .driver_info = BOARD_503_ISL3861},
	/* AWL-300 */
	{USB_DEVICE(VID_BENQ, 0x9000), .driver_info = BOARD_503_ISL3861},
	/* AWU-120, Compex WLU11 */
	{USB_DEVICE(VID_ADDTRON, 0xff31), .driver_info = BOARD_503_ISL3861},
	/* AP310 AnyPoint II USB */
	{USB_DEVICE(VID_INTEL, 0x0200), .driver_info = BOARD_503_ISL3861},
	/* Dynalink L11U */
	{USB_DEVICE(VID_GLOBAL_SUN, 0x7100), .driver_info = BOARD_503_ISL3861},
	/* Arescom WL-210, FCC id 07J-GL2411USB */
	{USB_DEVICE(VID_GLOBAL_SUN, 0x7110), .driver_info = BOARD_503_ISL3861},
	/* IO-DATA WN-B11/USB */
	{USB_DEVICE(VID_IO_DATA, 0x0919), .driver_info = BOARD_503_ISL3861},
	/* BT Voyager 1010 */
	{USB_DEVICE(VID_ASKEY, 0x0821), .driver_info = BOARD_503_ISL3861},
	/* at76c503-i3863 */
	/* Generic AT76C503/3863 device */
	{USB_DEVICE(VID_ATMEL, 0x7604), .driver_info = BOARD_503_ISL3863},
	/* Samsung SWL-2100U */
	{USB_DEVICE(VID_SAMSUNG, 0xa000), .driver_info = BOARD_503_ISL3863},
	/*
	 * at76c503-rfmd
	 */
	/* Generic AT76C503/RFMD device */
	{USB_DEVICE(VID_ATMEL, 0x7605), .driver_info = BOARD_503},
	/* Dynalink/Askey WLL013 (rfmd) */
	{USB_DEVICE(VID_ASKEY, 0x0321), .driver_info = BOARD_503},
	/* Linksys WUSB11 v2.6 */
	{USB_DEVICE(VID_LINKSYS, 0x2219), .driver_info = BOARD_503},
	/* Network Everywhere NWU11B */
	{USB_DEVICE(VID_LINKSYS, 0x2227), .driver_info = BOARD_503},
	/* Netgear MA 101 Rev. B */
	{USB_DEVICE(VID_NETGEAR, 0x4102), .driver_info = BOARD_503},
	/* DWL-120 rev. E */
	{USB_DEVICE(VID_DLINK, 0x3200), .driver_info = BOARD_503},
	/* Actiontec 802UAT1, HWU01150-01UK */
	{USB_DEVICE(VID_ACTIONTEC, 0x7605), .driver_info = BOARD_503},
	/* AirVast W-Buddie WN210 */
	{USB_DEVICE(VID_ATMEL, 0x4102), .driver_info = BOARD_503},
	/* XH1153 802.11b USB adapter */
	{USB_DEVICE(VID_DICK_SMITH, 0x5743), .driver_info = BOARD_503},
	/* WL-200U */
	{USB_DEVICE(VID_DICK_SMITH, 0x0002), .driver_info = BOARD_503},
	/* BenQ AWL-400 USB stick */
	{USB_DEVICE(VID_BENQ, 0x9001), .driver_info = BOARD_503},
	/* 3COM 3CRSHEW696 */
	{USB_DEVICE(VID_3COM, 0x0a01), .driver_info = BOARD_503},
	/* Siemens Santis ADSL WLAN USB adapter WLL 013 */
	{USB_DEVICE(VID_SIEMENS, 0x001b), .driver_info = BOARD_503},
	/* Belkin F5D6050, version 2 */
	{USB_DEVICE(VID_BELKIN_2, 0x0050), .driver_info = BOARD_503},
	/* iBlitzz, BWU613 (not *B or *SB) */
	{USB_DEVICE(VID_BLITZ, 0xb000), .driver_info = BOARD_503},
	/* Gigabyte GN-WLBM101 */
	{USB_DEVICE(VID_GIGABYTE, 0x8003), .driver_info = BOARD_503},
	/* Planex GW-US11S */
	{USB_DEVICE(VID_PLANEX, 0x3220), .driver_info = BOARD_503},
	/* Internal WLAN adapter in h5[4,5]xx series iPAQs */
	{USB_DEVICE(VID_COMPAQ, 0x0032), .driver_info = BOARD_503},
	/*
	 * at76c503-rfmd-acc
	 */
	/* SMC 2664W */
	{USB_DEVICE(VID_SMC, 0x3501), .driver_info = BOARD_503_ACC},
	/* Belkin F5D6050, SMC 2662W v2, SMC 2662W-AR */
	{USB_DEVICE(VID_BELKIN, 0xa002), .driver_info = BOARD_503_ACC},
	/*
	 * at76c505-rfmd
	 */
	/* Generic AT76C505/RFMD */
	{USB_DEVICE(VID_ATMEL, 0x7606), .driver_info = BOARD_505},
	/*
	 * at76c505-rfmd2958
	 */
	/* Generic AT76C505/RFMD, OvisLink WL-1130USB */
	{USB_DEVICE(VID_ATMEL, 0x7613), .driver_info = BOARD_505_2958},
	/* Fiberline WL-240U */
	{USB_DEVICE(VID_CNET, 0x0014), .driver_info = BOARD_505_2958},
	/* CNet CNUSB 611G */
	{USB_DEVICE(VID_CNET, 0x0013), .driver_info = BOARD_505_2958},
	/* Linksys WUSB11 v2.8 */
	{USB_DEVICE(VID_LINKSYS_1915, 0x2233), .driver_info = BOARD_505_2958},
	/* Xterasys XN-2122B, IBlitzz BWU613B/BWU613SB */
	{USB_DEVICE(VID_XTERASYS, 0x1001), .driver_info = BOARD_505_2958},
	/* Corega WLAN USB Stick 11 */
	{USB_DEVICE(VID_COREGA, 0x7613), .driver_info = BOARD_505_2958},
	/* Microstar MSI Box MS6978 */
	{USB_DEVICE(VID_MSI, 0x1020), .driver_info = BOARD_505_2958},
	/*
	 * at76c505a-rfmd2958
	 */
	/* Generic AT76C505A device */
	{USB_DEVICE(VID_ATMEL, 0x7614), .driver_info = BOARD_505A_2958},
	/* Generic AT76C505AS device */
	{USB_DEVICE(VID_ATMEL, 0x7617), .driver_info = BOARD_505A_2958},
	/* Siemens Gigaset USB WLAN Adapter 11 */
	{USB_DEVICE(VID_GIGASET, 0x0701), .driver_info = BOARD_505A_2958},
	/*
	 * at76c505amx-rfmd
	 */
	/* Generic AT76C505AMX device */
	{USB_DEVICE(VID_ATMEL, 0x7615), .driver_info = BOARD_505AMX},
	{}
};

MODULE_DEVICE_TABLE(usb, dev_table);

/* Supported rates of this hardware, bit 7 marks basic rates */
static const u8 hw_rates[] = { 0x82, 0x84, 0x0b, 0x16 };

/* Frequency of each channel in MHz */
static const long channel_frequency[] = {
	2412, 2417, 2422, 2427, 2432, 2437, 2442,
	2447, 2452, 2457, 2462, 2467, 2472, 2484
};

static const char *const preambles[] = { "long", "short", "auto" };

static const char *const mac_states[] = {
	[MAC_INIT] = "INIT",
	[MAC_SCANNING] = "SCANNING",
	[MAC_AUTH] = "AUTH",
	[MAC_ASSOC] = "ASSOC",
	[MAC_JOINING] = "JOINING",
	[MAC_CONNECTED] = "CONNECTED",
	[MAC_OWN_IBSS] = "OWN_IBSS"
};

#define NUM_CHANNELS ARRAY_SIZE(channel_frequency)

/* Firmware download */
/* DFU states */
#define STATE_IDLE			0x00
#define STATE_DETACH			0x01
#define STATE_DFU_IDLE			0x02
#define STATE_DFU_DOWNLOAD_SYNC		0x03
#define STATE_DFU_DOWNLOAD_BUSY		0x04
#define STATE_DFU_DOWNLOAD_IDLE		0x05
#define STATE_DFU_MANIFEST_SYNC		0x06
#define STATE_DFU_MANIFEST		0x07
#define STATE_DFU_MANIFEST_WAIT_RESET	0x08
#define STATE_DFU_UPLOAD_IDLE		0x09
#define STATE_DFU_ERROR			0x0a

/* DFU commands */
#define DFU_DETACH			0
#define DFU_DNLOAD			1
#define DFU_UPLOAD			2
#define DFU_GETSTATUS			3
#define DFU_CLRSTATUS			4
#define DFU_GETSTATE			5
#define DFU_ABORT			6

#define DFU_PACKETSIZE 1024

struct dfu_status {
	unsigned char status;
	unsigned char poll_timeout[3];
	unsigned char state;
	unsigned char string;
} __attribute__((packed));

/* Load a block of the first (internal) part of the firmware */
static int at76_load_int_fw_block(struct usb_device *udev, int blockno,
				  void *block, int size)
{
	int ret;

	at76_dbg(DBG_DFU, "%s(): block=%p, size=%d, blockno=%d", __func__,
		 block, size, blockno);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), DFU_DNLOAD,
			      USB_TYPE_CLASS | USB_DIR_OUT |
			      USB_RECIP_INTERFACE, blockno, 0, block, size,
			      USB_CTRL_GET_TIMEOUT);
	return ret;
}

static int at76_dfu_get_status(struct usb_device *udev,
			       struct dfu_status *status)
{
	int ret;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), DFU_GETSTATUS,
			      USB_TYPE_CLASS | USB_DIR_IN | USB_RECIP_INTERFACE,
			      0, 0, status, sizeof(struct dfu_status),
			      USB_CTRL_GET_TIMEOUT);
	return ret;
}

static u8 at76_dfu_get_state(struct usb_device *udev, u8 *state)
{
	int ret;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), DFU_GETSTATE,
			      USB_TYPE_CLASS | USB_DIR_IN | USB_RECIP_INTERFACE,
			      0, 0, state, 1, USB_CTRL_GET_TIMEOUT);
	return ret;
}

static inline u32 at76_get_timeout(struct dfu_status *s)
{
	u32 ret = (s->poll_timeout[2] << 16) | (s->poll_timeout[1] << 8) |
	    (s->poll_timeout[0]);

	return ret;
}

/* Load internal firmware from the buffer.  If manifest_sync_timeout > 0, use
 * its value in msec in the MANIFEST_SYNC state.  */
static int at76_usbdfu_download(struct usb_device *udev, u8 *dfu_buffer,
				u32 dfu_len, int manifest_sync_timeout)
{
	u8 *block;
	struct dfu_status dfu_stat_buf;
	int ret = 0;
	int need_dfu_state = 1;
	int is_done = 0;
	u8 dfu_state = 0;
	u32 dfu_timeout = 0;
	int dfu_block_bytes = 0;
	int dfu_bytes_left = dfu_len;
	int dfu_buffer_offset = 0;
	int dfu_block_cnt = 0;

	at76_dbg(DBG_DFU, "%s( %p, %u, %d)", __func__, dfu_buffer,
		 dfu_len, manifest_sync_timeout);

	if (dfu_len == 0) {
		err("FW Buffer length invalid!");
		return -EINVAL;
	}

	block = kmalloc(DFU_PACKETSIZE, GFP_KERNEL);
	if (block == NULL)
		return -ENOMEM;

	do {
		if (need_dfu_state) {
			ret = at76_dfu_get_state(udev, &dfu_state);
			if (ret < 0) {
				err("DFU: Failed to get DFU state: %d", ret);
				goto exit;
			}
			need_dfu_state = 0;
		}

		switch (dfu_state) {
		case STATE_DFU_DOWNLOAD_SYNC:
			at76_dbg(DBG_DFU, "STATE_DFU_DOWNLOAD_SYNC");
			ret = at76_dfu_get_status(udev, &dfu_stat_buf);
			if (ret >= 0) {
				dfu_state = dfu_stat_buf.state;
				dfu_timeout = at76_get_timeout(&dfu_stat_buf);
				need_dfu_state = 0;
			} else
				err("at76_dfu_get_status failed with %d", ret);
			break;

		case STATE_DFU_DOWNLOAD_BUSY:
			at76_dbg(DBG_DFU, "STATE_DFU_DOWNLOAD_BUSY");
			need_dfu_state = 1;

			at76_dbg(DBG_DFU, "DFU: Resetting device");
			schedule_timeout_interruptible(msecs_to_jiffies
						       (dfu_timeout));
			break;

		case STATE_DFU_DOWNLOAD_IDLE:
			at76_dbg(DBG_DFU, "DOWNLOAD...");
			/* fall through */
		case STATE_DFU_IDLE:
			at76_dbg(DBG_DFU, "DFU IDLE");

			dfu_block_bytes = min(dfu_bytes_left, DFU_PACKETSIZE);
			dfu_bytes_left -= dfu_block_bytes;
			memcpy(block, dfu_buffer + dfu_buffer_offset,
			       dfu_block_bytes);
			ret = at76_load_int_fw_block(udev, dfu_block_cnt, block,
						     dfu_block_bytes);
			dfu_buffer_offset += dfu_block_bytes;
			dfu_block_cnt++;

			if (ret < 0)
				err("dfu_download_block failed with %d", ret);
			need_dfu_state = 1;
			break;

		case STATE_DFU_MANIFEST_SYNC:
			at76_dbg(DBG_DFU, "STATE_DFU_MANIFEST_SYNC");

			ret = at76_dfu_get_status(udev, &dfu_stat_buf);
			if (ret < 0)
				break;

			dfu_state = dfu_stat_buf.state;
			dfu_timeout = at76_get_timeout(&dfu_stat_buf);
			need_dfu_state = 0;

			/* override the timeout from the status response,
			   needed for AT76C505A */
			if (manifest_sync_timeout > 0)
				dfu_timeout = manifest_sync_timeout;

			at76_dbg(DBG_DFU, "DFU: Waiting for manifest phase");
			schedule_timeout_interruptible(msecs_to_jiffies
						       (dfu_timeout));
			break;

		case STATE_DFU_MANIFEST:
			at76_dbg(DBG_DFU, "STATE_DFU_MANIFEST");
			is_done = 1;
			break;

		case STATE_DFU_MANIFEST_WAIT_RESET:
			at76_dbg(DBG_DFU, "STATE_DFU_MANIFEST_WAIT_RESET");
			is_done = 1;
			break;

		case STATE_DFU_UPLOAD_IDLE:
			at76_dbg(DBG_DFU, "STATE_DFU_UPLOAD_IDLE");
			break;

		case STATE_DFU_ERROR:
			at76_dbg(DBG_DFU, "STATE_DFU_ERROR");
			ret = -EPIPE;
			break;

		default:
			at76_dbg(DBG_DFU, "DFU UNKNOWN STATE (%d)", dfu_state);
			ret = -EINVAL;
			break;
		}
	} while (!is_done && (ret >= 0));

exit:
	kfree(block);
	if (ret >= 0)
		ret = 0;

	return ret;
}

/* Report that the scan results are ready */
static inline void at76_iwevent_scan_complete(struct net_device *netdev)
{
	union iwreq_data wrqu;
	wrqu.data.length = 0;
	wrqu.data.flags = 0;
	wireless_send_event(netdev, SIOCGIWSCAN, &wrqu, NULL);
	at76_dbg(DBG_WE_EVENTS, "%s: SIOCGIWSCAN sent", netdev->name);
}

static inline void at76_iwevent_bss_connect(struct net_device *netdev,
					    u8 *bssid)
{
	union iwreq_data wrqu;
	wrqu.data.length = 0;
	wrqu.data.flags = 0;
	memcpy(wrqu.ap_addr.sa_data, bssid, ETH_ALEN);
	wrqu.ap_addr.sa_family = ARPHRD_ETHER;
	wireless_send_event(netdev, SIOCGIWAP, &wrqu, NULL);
	at76_dbg(DBG_WE_EVENTS, "%s: %s: SIOCGIWAP sent", netdev->name,
		 __func__);
}

static inline void at76_iwevent_bss_disconnect(struct net_device *netdev)
{
	union iwreq_data wrqu;
	wrqu.data.length = 0;
	wrqu.data.flags = 0;
	memset(wrqu.ap_addr.sa_data, '\0', ETH_ALEN);
	wrqu.ap_addr.sa_family = ARPHRD_ETHER;
	wireless_send_event(netdev, SIOCGIWAP, &wrqu, NULL);
	at76_dbg(DBG_WE_EVENTS, "%s: %s: SIOCGIWAP sent", netdev->name,
		 __func__);
}

#define HEX2STR_BUFFERS 4
#define HEX2STR_MAX_LEN 64
#define BIN2HEX(x) ((x) < 10 ? '0' + (x) : (x) + 'A' - 10)

/* Convert binary data into hex string */
static char *hex2str(void *buf, int len)
{
	static atomic_t a = ATOMIC_INIT(0);
	static char bufs[3 * HEX2STR_MAX_LEN + 1][HEX2STR_BUFFERS];
	char *ret = bufs[atomic_inc_return(&a) & (HEX2STR_BUFFERS - 1)];
	char *obuf = ret;
	u8 *ibuf = buf;

	if (len > HEX2STR_MAX_LEN)
		len = HEX2STR_MAX_LEN;

	if (len <= 0) {
		ret[0] = '\0';
		return ret;
	}

	while (len--) {
		*obuf++ = BIN2HEX(*ibuf >> 4);
		*obuf++ = BIN2HEX(*ibuf & 0xf);
		*obuf++ = '-';
		ibuf++;
	}
	*(--obuf) = '\0';

	return ret;
}

#define MAC2STR_BUFFERS 4

static inline char *mac2str(u8 *mac)
{
	static atomic_t a = ATOMIC_INIT(0);
	static char bufs[6 * 3][MAC2STR_BUFFERS];
	char *str;

	str = bufs[atomic_inc_return(&a) & (MAC2STR_BUFFERS - 1)];
	sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return str;
}

/* LED trigger */
static int tx_activity;
static void at76_ledtrig_tx_timerfunc(unsigned long data);
static DEFINE_TIMER(ledtrig_tx_timer, at76_ledtrig_tx_timerfunc, 0, 0);
DEFINE_LED_TRIGGER(ledtrig_tx);

static void at76_ledtrig_tx_timerfunc(unsigned long data)
{
	static int tx_lastactivity;

	if (tx_lastactivity != tx_activity) {
		tx_lastactivity = tx_activity;
		led_trigger_event(ledtrig_tx, LED_FULL);
		mod_timer(&ledtrig_tx_timer, jiffies + msecs_to_jiffies(250));
	} else
		led_trigger_event(ledtrig_tx, LED_OFF);
}

static void at76_ledtrig_tx_activity(void)
{
	tx_activity++;
	if (!timer_pending(&ledtrig_tx_timer))
		mod_timer(&ledtrig_tx_timer, jiffies + msecs_to_jiffies(250));
}

/* Check if the given ssid is hidden */
static inline int at76_is_hidden_ssid(u8 *ssid, int length)
{
	static const u8 zeros[32];

	return (length == 0) ||
	    (length == 1 && *ssid == ' ') ||
	    (length > 0 && !memcmp(ssid, zeros, length));
}

static inline void at76_free_bss_list(struct at76_priv *priv)
{
	struct list_head *next, *ptr;
	unsigned long flags;

	spin_lock_irqsave(&priv->bss_list_spinlock, flags);

	priv->curr_bss = NULL;

	list_for_each_safe(ptr, next, &priv->bss_list) {
		list_del(ptr);
		kfree(list_entry(ptr, struct bss_info, list));
	}

	spin_unlock_irqrestore(&priv->bss_list_spinlock, flags);
}

static int at76_remap(struct usb_device *udev)
{
	int ret;
	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x0a,
			      USB_TYPE_VENDOR | USB_DIR_OUT |
			      USB_RECIP_INTERFACE, 0, 0, NULL, 0,
			      USB_CTRL_GET_TIMEOUT);
	if (ret < 0)
		return ret;
	return 0;
}

static int at76_get_op_mode(struct usb_device *udev)
{
	int ret;
	u8 op_mode;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x33,
			      USB_TYPE_VENDOR | USB_DIR_IN |
			      USB_RECIP_INTERFACE, 0x01, 0, &op_mode, 1,
			      USB_CTRL_GET_TIMEOUT);
	if (ret < 0)
		return ret;
	return op_mode;
}

/* Load a block of the second ("external") part of the firmware */
static inline int at76_load_ext_fw_block(struct usb_device *udev, int blockno,
					 void *block, int size)
{
	return usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x0e,
			       USB_TYPE_VENDOR | USB_DIR_OUT | USB_RECIP_DEVICE,
			       0x0802, blockno, block, size,
			       USB_CTRL_GET_TIMEOUT);
}

static inline int at76_get_hw_cfg(struct usb_device *udev,
				  union at76_hwcfg *buf, int buf_size)
{
	return usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x33,
			       USB_TYPE_VENDOR | USB_DIR_IN |
			       USB_RECIP_INTERFACE, ((0x0a << 8) | 0x02), 0,
			       buf, buf_size, USB_CTRL_GET_TIMEOUT);
}

/* Intersil boards use a different "value" for GetHWConfig requests */
static inline int at76_get_hw_cfg_intersil(struct usb_device *udev,
					   union at76_hwcfg *buf, int buf_size)
{
	return usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x33,
			       USB_TYPE_VENDOR | USB_DIR_IN |
			       USB_RECIP_INTERFACE, ((0x09 << 8) | 0x02), 0,
			       buf, buf_size, USB_CTRL_GET_TIMEOUT);
}

/* Get the hardware configuration for the adapter and put it to the appropriate
 * fields of 'priv' (the GetHWConfig request and interpretation of the result
 * depends on the board type) */
static int at76_get_hw_config(struct at76_priv *priv)
{
	int ret;
	union at76_hwcfg *hwcfg = kmalloc(sizeof(*hwcfg), GFP_KERNEL);

	if (!hwcfg)
		return -ENOMEM;

	switch (priv->board_type) {

	case BOARD_503_ISL3861:
	case BOARD_503_ISL3863:
		ret = at76_get_hw_cfg_intersil(priv->udev, hwcfg,
					       sizeof(hwcfg->i));
		if (ret < 0)
			break;
		memcpy(priv->mac_addr, hwcfg->i.mac_addr, ETH_ALEN);
		priv->regulatory_domain = hwcfg->i.regulatory_domain;
		break;

	case BOARD_503:
	case BOARD_503_ACC:
		ret = at76_get_hw_cfg(priv->udev, hwcfg, sizeof(hwcfg->r3));
		if (ret < 0)
			break;
		memcpy(priv->mac_addr, hwcfg->r3.mac_addr, ETH_ALEN);
		priv->regulatory_domain = hwcfg->r3.regulatory_domain;
		break;

	case BOARD_505:
	case BOARD_505_2958:
	case BOARD_505A_2958:
		ret = at76_get_hw_cfg(priv->udev, hwcfg, sizeof(hwcfg->r5));
		if (ret < 0)
			break;
		memcpy(priv->mac_addr, hwcfg->r5.mac_addr, ETH_ALEN);
		priv->regulatory_domain = hwcfg->r5.regulatory_domain;
		break;

	default:
		err("Bad board type set (%d).  Unable to get hardware config.",
		    priv->board_type);
		ret = -EINVAL;
	}

	kfree(hwcfg);

	if (ret < 0)
		err("Get HW Config failed (%d)", ret);

	return ret;
}

static struct reg_domain const *at76_get_reg_domain(u16 code)
{
	static struct reg_domain const fd_tab[] = {
		{0x10, "FCC (USA)", 0x7ff},	/* ch 1-11 */
		{0x20, "IC (Canada)", 0x7ff},	/* ch 1-11 */
		{0x30, "ETSI (most of Europe)", 0x1fff},	/* ch 1-13 */
		{0x31, "Spain", 0x600},	/* ch 10-11 */
		{0x32, "France", 0x1e00},	/* ch 10-13 */
		{0x40, "MKK (Japan)", 0x2000},	/* ch 14 */
		{0x41, "MKK1 (Japan)", 0x3fff},	/* ch 1-14 */
		{0x50, "Israel", 0x3fc},	/* ch 3-9 */
	};
	static int const tab_len = ARRAY_SIZE(fd_tab);

	/* use this if an unknown code comes in */
	static struct reg_domain const unknown = { 0, "<unknown>", 0xffffffff };

	int i;

	for (i = 0; i < tab_len; i++)
		if (code == fd_tab[i].code)
			break;

	return (i >= tab_len) ? &unknown : &fd_tab[i];
}

static inline int at76_get_mib(struct usb_device *udev, u16 mib, void *buf,
			       int buf_size)
{
	return usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x33,
			       USB_TYPE_VENDOR | USB_DIR_IN |
			       USB_RECIP_INTERFACE, mib << 8, 0, buf, buf_size,
			       USB_CTRL_GET_TIMEOUT);
}

/* Return positive number for status, negative for an error */
static inline int at76_get_cmd_status(struct usb_device *udev, u8 cmd)
{
	u8 stat_buf[40];
	int ret;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x22,
			      USB_TYPE_VENDOR | USB_DIR_IN |
			      USB_RECIP_INTERFACE, cmd, 0, stat_buf,
			      sizeof(stat_buf), USB_CTRL_GET_TIMEOUT);
	if (ret < 0)
		return ret;

	return stat_buf[5];
}

#define EXT_FW_BLOCK_SIZE 1024
static int at76_download_external_fw(struct usb_device *udev, u8 *buf, int size)
{
	int i = 0;
	int ret;
	u8 *block;

	if (size < 0)
		return -EINVAL;
	if ((size > 0) && (buf == NULL))
		return -EFAULT;

	block = kmalloc(EXT_FW_BLOCK_SIZE, GFP_KERNEL);
	if (block == NULL)
		return -ENOMEM;

	at76_dbg(DBG_DEVSTART, "downloading external firmware");

	while (size > 0) {
		int bsize = size > EXT_FW_BLOCK_SIZE ? EXT_FW_BLOCK_SIZE : size;

		memcpy(block, buf, bsize);
		at76_dbg(DBG_DEVSTART,
			 "ext fw, size left = %5d, bsize = %4d, i = %2d",
			 size, bsize, i);
		ret = at76_load_ext_fw_block(udev, i, block, bsize);
		if (ret < 0) {
			err("loading %dth firmware block failed: %d", i, ret);
			goto exit;
		}
		buf += bsize;
		size -= bsize;
		i++;
	}

	/* for fw >= 0.100, the device needs
	   an extra empty block: */
	ret = at76_load_ext_fw_block(udev, i, block, 0);
	if (ret < 0) {
		err("loading %dth firmware block failed: %d", ret, i);
		goto exit;
	}

exit:
	kfree(block);
	return ret;
}

static int at76_set_card_command(struct usb_device *udev, int cmd, void *buf,
				 int buf_size)
{
	int ret;
	struct at76_command *cmd_buf = kmalloc(sizeof(struct at76_command) +
					       buf_size, GFP_KERNEL);

	if (!cmd_buf)
		return -ENOMEM;

	cmd_buf->cmd = cmd;
	cmd_buf->reserved = 0;
	cmd_buf->size = cpu_to_le16(buf_size);
	memcpy(cmd_buf->data, buf, buf_size);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x0e,
			      USB_TYPE_VENDOR | USB_DIR_OUT | USB_RECIP_DEVICE,
			      0, 0, cmd_buf,
			      sizeof(struct at76_command) + buf_size,
			      USB_CTRL_GET_TIMEOUT);
	kfree(cmd_buf);
	return ret;
}

#define MAKE_CMD_STATUS_CASE(c)	case (c): return #c
static const char *at76_get_cmd_status_string(u8 cmd_status)
{
	switch (cmd_status) {
		MAKE_CMD_STATUS_CASE(CMD_STATUS_IDLE);
		MAKE_CMD_STATUS_CASE(CMD_STATUS_COMPLETE);
		MAKE_CMD_STATUS_CASE(CMD_STATUS_UNKNOWN);
		MAKE_CMD_STATUS_CASE(CMD_STATUS_INVALID_PARAMETER);
		MAKE_CMD_STATUS_CASE(CMD_STATUS_FUNCTION_NOT_SUPPORTED);
		MAKE_CMD_STATUS_CASE(CMD_STATUS_TIME_OUT);
		MAKE_CMD_STATUS_CASE(CMD_STATUS_IN_PROGRESS);
		MAKE_CMD_STATUS_CASE(CMD_STATUS_HOST_FAILURE);
		MAKE_CMD_STATUS_CASE(CMD_STATUS_SCAN_FAILED);
	}

	return "UNKNOWN";
}

/* Wait until the command is completed */
static int at76_wait_completion(struct at76_priv *priv, int cmd)
{
	struct net_device *netdev = priv->netdev;
	int status = 0;
	unsigned long timeout = jiffies + CMD_COMPLETION_TIMEOUT;

	do {
		status = at76_get_cmd_status(priv->udev, cmd);
		if (status < 0) {
			err("%s: at76_get_cmd_status failed: %d", netdev->name,
			    status);
			break;
		}

		at76_dbg(DBG_WAIT_COMPLETE,
			 "%s: Waiting on cmd %d, status = %d (%s)",
			 priv->netdev->name, cmd, status,
			 at76_get_cmd_status_string(status));

		if (status != CMD_STATUS_IN_PROGRESS
		    && status != CMD_STATUS_IDLE)
			break;

		schedule_timeout_interruptible(HZ / 10);	/* 100 ms */
		if (time_after(jiffies, timeout)) {
			err("%s: timeout waiting for cmd %d completion",
			    netdev->name, cmd);
			status = -ETIMEDOUT;
			break;
		}
	} while (1);

	return status;
}

static int at76_set_mib(struct at76_priv *priv, struct set_mib_buffer *buf)
{
	int ret;

	ret = at76_set_card_command(priv->udev, CMD_SET_MIB, buf,
				    offsetof(struct set_mib_buffer,
					     data) + buf->size);
	if (ret < 0)
		return ret;

	ret = at76_wait_completion(priv, CMD_SET_MIB);
	if (ret != CMD_STATUS_COMPLETE) {
		printk(KERN_INFO
		       "%s: set_mib: at76_wait_completion failed "
		       "with %d\n", priv->netdev->name, ret);
		ret = -EIO;
	}

	return ret;
}

/* Return < 0 on error, == 0 if no command sent, == 1 if cmd sent */
static int at76_set_radio(struct at76_priv *priv, int on_off)
{
	int ret;

	if (priv->radio_on == on_off)
		return 0;

	ret = at76_set_card_command(priv->udev, CMD_RADIO, NULL, 0);
	if (ret < 0)
		err("%s: at76_set_card_command(CMD_RADIO) failed: %d",
		    priv->netdev->name, ret);
	else
		ret = 1;

	priv->radio_on = on_off;
	return ret;
}

/* Set current power save mode (AT76_PM_OFF/AT76_PM_ON/AT76_PM_SMART) */
static int at76_set_pm_mode(struct at76_priv *priv)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC_MGMT;
	priv->mib_buf.size = 1;
	priv->mib_buf.index = offsetof(struct mib_mac_mgmt, power_mgmt_mode);

	priv->mib_buf.data[0] = priv->pm_mode;

	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (pm_mode) failed: %d", priv->netdev->name,
		    ret);

	return ret;
}

/* Set the association id for power save mode */
static int at76_set_associd(struct at76_priv *priv, u16 id)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC_MGMT;
	priv->mib_buf.size = 2;
	priv->mib_buf.index = offsetof(struct mib_mac_mgmt, station_id);

	*(__le16 *)priv->mib_buf.data = cpu_to_le16(id);

	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (associd) failed: %d", priv->netdev->name,
		    ret);

	return ret;
}

/* Set the listen interval for power save mode */
static int at76_set_listen_interval(struct at76_priv *priv, u16 interval)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC;
	priv->mib_buf.size = 2;
	priv->mib_buf.index = offsetof(struct mib_mac, listen_interval);

	*(__le16 *)priv->mib_buf.data = cpu_to_le16(interval);

	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (listen_interval) failed: %d",
		    priv->netdev->name, ret);

	return ret;
}

static int at76_set_preamble(struct at76_priv *priv, u8 type)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_LOCAL;
	priv->mib_buf.size = 1;
	priv->mib_buf.index = offsetof(struct mib_local, preamble_type);
	priv->mib_buf.data[0] = type;
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (preamble) failed: %d", priv->netdev->name,
		    ret);

	return ret;
}

static int at76_set_frag(struct at76_priv *priv, u16 size)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC;
	priv->mib_buf.size = 2;
	priv->mib_buf.index = offsetof(struct mib_mac, frag_threshold);
	*(__le16 *)priv->mib_buf.data = cpu_to_le16(size);
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (frag threshold) failed: %d",
		    priv->netdev->name, ret);

	return ret;
}

static int at76_set_rts(struct at76_priv *priv, u16 size)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC;
	priv->mib_buf.size = 2;
	priv->mib_buf.index = offsetof(struct mib_mac, rts_threshold);
	*(__le16 *)priv->mib_buf.data = cpu_to_le16(size);
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (rts) failed: %d", priv->netdev->name, ret);

	return ret;
}

static int at76_set_autorate_fallback(struct at76_priv *priv, int onoff)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_LOCAL;
	priv->mib_buf.size = 1;
	priv->mib_buf.index = offsetof(struct mib_local, txautorate_fallback);
	priv->mib_buf.data[0] = onoff;
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (autorate fallback) failed: %d",
		    priv->netdev->name, ret);

	return ret;
}

/* Set network device type for the current mode */
static void at76_set_monitor_mode(struct at76_priv *priv)
{
	if (priv->iw_mode == IW_MODE_MONITOR) {
		at76_dbg(DBG_MONITOR_MODE, "%s: MONITOR MODE ON",
			 priv->netdev->name);
		priv->netdev->type = ARPHRD_IEEE80211_RADIOTAP;
	} else {
		at76_dbg(DBG_MONITOR_MODE, "%s: MONITOR MODE OFF",
			 priv->netdev->name);
		priv->netdev->type = ARPHRD_ETHER;
	}
}

static int at76_add_mac_address(struct at76_priv *priv, void *addr)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC_ADD;
	priv->mib_buf.size = ETH_ALEN;
	priv->mib_buf.index = offsetof(struct mib_mac_addr, mac_addr);
	memcpy(priv->mib_buf.data, addr, ETH_ALEN);
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (MAC_ADDR, mac_addr) failed: %d",
		    priv->netdev->name, ret);

	return ret;
}

#if 0
/* Implemented to get promisc. mode working, but does not help.
   May still be useful for multicast eventually. */
static int at76_set_group_address(struct at76_priv *priv, u8 *addr, int n)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC_ADD;
	priv->mib_buf.size = ETH_ALEN;
	priv->mib_buf.index =
	    offsetof(struct mib_mac_addr, group_addr) + n * ETH_ALEN;
	memcpy(priv->mib_buf.data, addr, ETH_ALEN);
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (MIB_MAC_ADD, group_addr) failed: %d",
		    priv->netdev->name, ret);

	/* I do not know anything about the group_addr_status field... (oku) */
	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC_ADD;
	priv->mib_buf.size = 1;
	priv->mib_buf.index =
	    offsetof(struct mib_mac_addr, group_addr_status) + n;
	priv->mib_buf.data[0] = 1;
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (MIB_MAC_ADD, group_addr_status) failed: %d",
		    priv->netdev->name, ret);

	return ret;
}
#endif

static int at76_dump_mib_mac_addr(struct at76_priv *priv)
{
	int ret = 0;
	struct mib_mac_addr *mac_addr =
	    kmalloc(sizeof(struct mib_mac_addr), GFP_KERNEL);

	if (!mac_addr) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = at76_get_mib(priv->udev, MIB_MAC_ADD,
			   mac_addr, sizeof(struct mib_mac_addr));
	if (ret < 0) {
		err("%s: at76_get_mib (MAC_ADDR) failed: %d",
		    priv->netdev->name, ret);
		goto error;
	}

	dbg("%s: MIB MAC_ADDR: mac_addr %s res 0x%x 0x%x group_addr %s status "
	    "%d %d %d %d", priv->netdev->name, mac2str(mac_addr->mac_addr),
	    mac_addr->res[0], mac_addr->res[1],
	    hex2str(mac_addr->group_addr, 4 * ETH_ALEN),
	    mac_addr->group_addr_status[0], mac_addr->group_addr_status[1],
	    mac_addr->group_addr_status[2], mac_addr->group_addr_status[3]);

error:
	kfree(mac_addr);
exit:
	return ret;
}

static int at76_dump_mib_mac_wep(struct at76_priv *priv)
{
	int ret = 0;
	char *defkey;
	struct mib_mac_wep *mac_wep =
	    kmalloc(sizeof(struct mib_mac_wep), GFP_KERNEL);

	if (!mac_wep) {
		ret = -ENOMEM;
		goto exit;
	}
	ret = at76_get_mib(priv->udev, MIB_MAC_WEP, mac_wep,
			   sizeof(struct mib_mac_wep));
	if (ret < 0) {
		err("%s: at76_get_mib (MAC_WEP) failed: %d", priv->netdev->name,
		    ret);
		goto error;
	}

	if (mac_wep->wep_default_key_id < 4)
		defkey =
		    hex2str(mac_wep->
			    wep_default_keyvalue[mac_wep->wep_default_key_id],
			    mac_wep->encryption_level == 2 ? 13 : 5);
	else
		defkey = "<invalid key id>";

	dbg("%s: MIB MAC_WEP: priv_invoked %u def_key_id %u key_len %u "
	    "excl_unencr %u wep_icv_err %u wep_excluded %u encr_level %u "
	    "key %d: %s", priv->netdev->name, mac_wep->privacy_invoked,
	    mac_wep->wep_default_key_id, mac_wep->wep_key_mapping_len,
	    mac_wep->exclude_unencrypted,
	    le32_to_cpu(mac_wep->wep_icv_error_count),
	    le32_to_cpu(mac_wep->wep_excluded_count), mac_wep->encryption_level,
	    mac_wep->wep_default_key_id, defkey);

error:
	kfree(mac_wep);
exit:
	return ret;
}

static int at76_dump_mib_mac_mgmt(struct at76_priv *priv)
{
	int ret = 0;
	struct mib_mac_mgmt *mac_mgmt =
	    kmalloc(sizeof(struct mib_mac_mgmt), GFP_KERNEL);
	char country_string[4];

	if (!mac_mgmt) {
		ret = -ENOMEM;
		goto exit;
	}
	ret = at76_get_mib(priv->udev, MIB_MAC_MGMT, mac_mgmt,
			   sizeof(struct mib_mac_mgmt));
	if (ret < 0) {
		err("%s: at76_get_mib failed: %d", priv->netdev->name, ret);
		goto error;
	}

	memcpy(&country_string, mac_mgmt->country_string, 3);
	country_string[3] = '\0';

	dbg("%s: MIB MAC_MGMT: beacon_period %d CFP_max_duration %d "
	    "medium_occupancy_limit %d station_id 0x%x ATIM_window %d "
	    "CFP_mode %d privacy_opt_impl %d DTIM_period %d CFP_period %d "
	    "current_bssid %s current_essid %s current_bss_type %d "
	    "pm_mode %d ibss_change %d res %d "
	    "multi_domain_capability_implemented %d "
	    "international_roaming %d country_string %s",
	    priv->netdev->name,
	    le16_to_cpu(mac_mgmt->beacon_period),
	    le16_to_cpu(mac_mgmt->CFP_max_duration),
	    le16_to_cpu(mac_mgmt->medium_occupancy_limit),
	    le16_to_cpu(mac_mgmt->station_id),
	    le16_to_cpu(mac_mgmt->ATIM_window),
	    mac_mgmt->CFP_mode,
	    mac_mgmt->privacy_option_implemented,
	    mac_mgmt->DTIM_period,
	    mac_mgmt->CFP_period,
	    mac2str(mac_mgmt->current_bssid),
	    hex2str(mac_mgmt->current_essid, IW_ESSID_MAX_SIZE),
	    mac_mgmt->current_bss_type,
	    mac_mgmt->power_mgmt_mode,
	    mac_mgmt->ibss_change,
	    mac_mgmt->res,
	    mac_mgmt->multi_domain_capability_implemented,
	    mac_mgmt->multi_domain_capability_enabled, country_string);
error:
	kfree(mac_mgmt);
exit:
	return ret;
}

static int at76_dump_mib_mac(struct at76_priv *priv)
{
	int ret = 0;
	struct mib_mac *mac = kmalloc(sizeof(struct mib_mac), GFP_KERNEL);

	if (!mac) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = at76_get_mib(priv->udev, MIB_MAC, mac, sizeof(struct mib_mac));
	if (ret < 0) {
		err("%s: at76_get_mib failed: %d", priv->netdev->name, ret);
		goto error;
	}

	dbg("%s: MIB MAC: max_tx_msdu_lifetime %d max_rx_lifetime %d "
	    "frag_threshold %d rts_threshold %d cwmin %d cwmax %d "
	    "short_retry_time %d long_retry_time %d scan_type %d "
	    "scan_channel %d probe_delay %u min_channel_time %d "
	    "max_channel_time %d listen_int %d desired_ssid %s "
	    "desired_bssid %s desired_bsstype %d",
	    priv->netdev->name,
	    le32_to_cpu(mac->max_tx_msdu_lifetime),
	    le32_to_cpu(mac->max_rx_lifetime),
	    le16_to_cpu(mac->frag_threshold),
	    le16_to_cpu(mac->rts_threshold),
	    le16_to_cpu(mac->cwmin),
	    le16_to_cpu(mac->cwmax),
	    mac->short_retry_time,
	    mac->long_retry_time,
	    mac->scan_type,
	    mac->scan_channel,
	    le16_to_cpu(mac->probe_delay),
	    le16_to_cpu(mac->min_channel_time),
	    le16_to_cpu(mac->max_channel_time),
	    le16_to_cpu(mac->listen_interval),
	    hex2str(mac->desired_ssid, IW_ESSID_MAX_SIZE),
	    mac2str(mac->desired_bssid), mac->desired_bsstype);
error:
	kfree(mac);
exit:
	return ret;
}

static int at76_dump_mib_phy(struct at76_priv *priv)
{
	int ret = 0;
	struct mib_phy *phy = kmalloc(sizeof(struct mib_phy), GFP_KERNEL);

	if (!phy) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = at76_get_mib(priv->udev, MIB_PHY, phy, sizeof(struct mib_phy));
	if (ret < 0) {
		err("%s: at76_get_mib failed: %d", priv->netdev->name, ret);
		goto error;
	}

	dbg("%s: MIB PHY: ed_threshold %d slot_time %d sifs_time %d "
	    "preamble_length %d plcp_header_length %d mpdu_max_length %d "
	    "cca_mode_supported %d operation_rate_set "
	    "0x%x 0x%x 0x%x 0x%x channel_id %d current_cca_mode %d "
	    "phy_type %d current_reg_domain %d",
	    priv->netdev->name,
	    le32_to_cpu(phy->ed_threshold),
	    le16_to_cpu(phy->slot_time),
	    le16_to_cpu(phy->sifs_time),
	    le16_to_cpu(phy->preamble_length),
	    le16_to_cpu(phy->plcp_header_length),
	    le16_to_cpu(phy->mpdu_max_length),
	    le16_to_cpu(phy->cca_mode_supported),
	    phy->operation_rate_set[0], phy->operation_rate_set[1],
	    phy->operation_rate_set[2], phy->operation_rate_set[3],
	    phy->channel_id,
	    phy->current_cca_mode, phy->phy_type, phy->current_reg_domain);
error:
	kfree(phy);
exit:
	return ret;
}

static int at76_dump_mib_local(struct at76_priv *priv)
{
	int ret = 0;
	struct mib_local *local = kmalloc(sizeof(struct mib_phy), GFP_KERNEL);

	if (!local) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = at76_get_mib(priv->udev, MIB_LOCAL, local,
			   sizeof(struct mib_local));
	if (ret < 0) {
		err("%s: at76_get_mib failed: %d", priv->netdev->name, ret);
		goto error;
	}

	dbg("%s: MIB PHY: beacon_enable %d txautorate_fallback %d "
	    "ssid_size %d promiscuous_mode %d preamble_type %d",
	    priv->netdev->name,
	    local->beacon_enable,
	    local->txautorate_fallback,
	    local->ssid_size, local->promiscuous_mode, local->preamble_type);
error:
	kfree(local);
exit:
	return ret;
}

static int at76_get_mib_mdomain(struct at76_priv *priv, struct mib_mdomain *val)
{
	int ret = 0;
	struct mib_mdomain *mdomain =
	    kmalloc(sizeof(struct mib_mdomain), GFP_KERNEL);

	if (!mdomain) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = at76_get_mib(priv->udev, MIB_MDOMAIN, mdomain,
			   sizeof(struct mib_mdomain));
	if (ret < 0) {
		err("%s: at76_get_mib failed: %d", priv->netdev->name, ret);
		goto error;
	}

	memcpy(val, mdomain, sizeof(*val));

error:
	kfree(mdomain);
exit:
	return ret;
}

static void at76_dump_mib_mdomain(struct at76_priv *priv)
{
	int ret;
	struct mib_mdomain mdomain;

	ret = at76_get_mib_mdomain(priv, &mdomain);
	if (ret < 0) {
		err("%s: at76_get_mib_mdomain returned %d", __func__, ret);
		return;
	}

	at76_dbg(DBG_MIB, "%s: MIB MDOMAIN: channel_list %s",
		 priv->netdev->name,
		 hex2str(mdomain.channel_list, sizeof(mdomain.channel_list)));

	at76_dbg(DBG_MIB, "%s: MIB MDOMAIN: tx_powerlevel %s",
		 priv->netdev->name,
		 hex2str(mdomain.tx_powerlevel, sizeof(mdomain.tx_powerlevel)));
}

static int at76_get_current_bssid(struct at76_priv *priv)
{
	int ret = 0;
	struct mib_mac_mgmt *mac_mgmt =
	    kmalloc(sizeof(struct mib_mac_mgmt), GFP_KERNEL);

	if (!mac_mgmt) {
		ret = -ENOMEM;
		goto exit;
	}

	ret = at76_get_mib(priv->udev, MIB_MAC_MGMT, mac_mgmt,
			   sizeof(struct mib_mac_mgmt));
	if (ret < 0) {
		err("%s: at76_get_mib failed: %d", priv->netdev->name, ret);
		goto error;
	}
	memcpy(priv->bssid, mac_mgmt->current_bssid, ETH_ALEN);
	printk(KERN_INFO "%s: using BSSID %s\n", priv->netdev->name,
	       mac2str(priv->bssid));
error:
	kfree(mac_mgmt);
exit:
	return ret;
}

static int at76_get_current_channel(struct at76_priv *priv)
{
	int ret = 0;
	struct mib_phy *phy = kmalloc(sizeof(struct mib_phy), GFP_KERNEL);

	if (!phy) {
		ret = -ENOMEM;
		goto exit;
	}
	ret = at76_get_mib(priv->udev, MIB_PHY, phy, sizeof(struct mib_phy));
	if (ret < 0) {
		err("%s: at76_get_mib(MIB_PHY) failed: %d", priv->netdev->name,
		    ret);
		goto error;
	}
	priv->channel = phy->channel_id;
error:
	kfree(phy);
exit:
	return ret;
}

/**
 * at76_start_scan - start a scan
 *
 * @use_essid - use the configured ESSID in non passive mode
 * @ir_step - international roaming step (0, 1)
 */
static int at76_start_scan(struct at76_priv *priv, int use_essid, int ir_step)
{
	struct at76_req_scan scan;

	memset(&scan, 0, sizeof(struct at76_req_scan));
	memset(scan.bssid, 0xff, ETH_ALEN);

	if (use_essid) {
		memcpy(scan.essid, priv->essid, IW_ESSID_MAX_SIZE);
		scan.essid_size = priv->essid_size;
	} else
		scan.essid_size = 0;

	/* jal: why should we start at a certain channel? we do scan the whole
	   range allowed by reg domain. */
	scan.channel = priv->channel;

	/* atmelwlandriver differs between scan type 0 and 1 (active/passive)
	   For ad-hoc mode, it uses type 0 only. */
	if (priv->international_roaming == IR_ON && ir_step == 0)
		scan.scan_type = SCAN_TYPE_PASSIVE;
	else
		scan.scan_type = priv->scan_mode;

	/* INFO: For probe_delay, not multiplying by 1024 as this will be
	   slightly less than min_channel_time
	   (per spec: probe delay < min. channel time) */
	scan.min_channel_time = cpu_to_le16(priv->scan_min_time);
	scan.max_channel_time = cpu_to_le16(priv->scan_max_time);
	scan.probe_delay = cpu_to_le16(priv->scan_min_time * 1000);

	if (priv->international_roaming == IR_ON && ir_step == 1)
		scan.international_scan = 0;
	else
		scan.international_scan = priv->international_roaming;

	/* other values are set to 0 for type 0 */

	at76_dbg(DBG_PROGRESS, "%s: start_scan (use_essid = %d, intl = %d, "
		 "channel = %d, probe_delay = %d, scan_min_time = %d, "
		 "scan_max_time = %d)",
		 priv->netdev->name, use_essid,
		 scan.international_scan, scan.channel,
		 le16_to_cpu(scan.probe_delay),
		 le16_to_cpu(scan.min_channel_time),
		 le16_to_cpu(scan.max_channel_time));

	return at76_set_card_command(priv->udev, CMD_SCAN, &scan, sizeof(scan));
}

/* Enable monitor mode */
static int at76_start_monitor(struct at76_priv *priv)
{
	struct at76_req_scan scan;
	int ret;

	memset(&scan, 0, sizeof(struct at76_req_scan));
	memset(scan.bssid, 0xff, ETH_ALEN);

	scan.channel = priv->channel;
	scan.scan_type = SCAN_TYPE_PASSIVE;
	scan.international_scan = priv->international_roaming;

	ret = at76_set_card_command(priv->udev, CMD_SCAN, &scan, sizeof(scan));
	if (ret >= 0)
		ret = at76_get_cmd_status(priv->udev, CMD_SCAN);

	return ret;
}

static int at76_start_ibss(struct at76_priv *priv)
{
	struct at76_req_ibss bss;
	int ret;

	WARN_ON(priv->mac_state != MAC_OWN_IBSS);
	if (priv->mac_state != MAC_OWN_IBSS)
		return -EBUSY;

	memset(&bss, 0, sizeof(struct at76_req_ibss));
	memset(bss.bssid, 0xff, ETH_ALEN);
	memcpy(bss.essid, priv->essid, IW_ESSID_MAX_SIZE);
	bss.essid_size = priv->essid_size;
	bss.bss_type = ADHOC_MODE;
	bss.channel = priv->channel;

	ret = at76_set_card_command(priv->udev, CMD_START_IBSS, &bss,
				    sizeof(struct at76_req_ibss));
	if (ret < 0) {
		err("%s: start_ibss failed: %d", priv->netdev->name, ret);
		return ret;
	}

	ret = at76_wait_completion(priv, CMD_START_IBSS);
	if (ret != CMD_STATUS_COMPLETE) {
		err("%s start_ibss failed to complete, %d",
		    priv->netdev->name, ret);
		return ret;
	}

	ret = at76_get_current_bssid(priv);
	if (ret < 0)
		return ret;

	ret = at76_get_current_channel(priv);
	if (ret < 0)
		return ret;

	/* not sure what this is good for ??? */
	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC_MGMT;
	priv->mib_buf.size = 1;
	priv->mib_buf.index = offsetof(struct mib_mac_mgmt, ibss_change);
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0) {
		err("%s: set_mib (ibss change ok) failed: %d",
		    priv->netdev->name, ret);
		return ret;
	}

	netif_carrier_on(priv->netdev);
	netif_start_queue(priv->netdev);
	return 0;
}

/* Request card to join BSS in managed or ad-hoc mode */
static int at76_join_bss(struct at76_priv *priv, struct bss_info *ptr)
{
	struct at76_req_join join;

	BUG_ON(ptr == NULL);

	memset(&join, 0, sizeof(struct at76_req_join));
	memcpy(join.bssid, ptr->bssid, ETH_ALEN);
	memcpy(join.essid, ptr->ssid, ptr->ssid_len);
	join.essid_size = ptr->ssid_len;
	join.bss_type = (priv->iw_mode == IW_MODE_ADHOC ? 1 : 2);
	join.channel = ptr->channel;
	join.timeout = cpu_to_le16(2000);

	at76_dbg(DBG_PROGRESS,
		 "%s join addr %s ssid %s type %d ch %d timeout %d",
		 priv->netdev->name, mac2str(join.bssid), join.essid,
		 join.bss_type, join.channel, le16_to_cpu(join.timeout));
	return at76_set_card_command(priv->udev, CMD_JOIN, &join,
				     sizeof(struct at76_req_join));
}

/* Calculate padding from txbuf->wlength (which excludes the USB TX header),
   likely to compensate a flaw in the AT76C503A USB part ... */
static inline int at76_calc_padding(int wlen)
{
	/* add the USB TX header */
	wlen += AT76_TX_HDRLEN;

	wlen = wlen % 64;

	if (wlen < 50)
		return 50 - wlen;

	if (wlen >= 61)
		return 64 + 50 - wlen;

	return 0;
}

/* We are doing a lot of things here in an interrupt. Need
   a bh handler (Watching TV with a TV card is probably
   a good test: if you see flickers, we are doing too much.
   Currently I do see flickers... even with our tasklet :-( )
   Maybe because the bttv driver and usb-uhci use the same interrupt
*/
/* Or maybe because our BH handler is preempting bttv's BH handler.. BHs don't
 * solve everything.. (alex) */
static void at76_read_bulk_callback(struct urb *urb)
{
	struct at76_priv *priv = urb->context;

	priv->rx_tasklet.data = (unsigned long)urb;
	tasklet_schedule(&priv->rx_tasklet);
	return;
}

static void at76_write_bulk_callback(struct urb *urb)
{
	struct at76_priv *priv = urb->context;
	struct net_device_stats *stats = &priv->stats;
	unsigned long flags;
	struct at76_tx_buffer *mgmt_buf;
	int ret;

	switch (urb->status) {
	case 0:
		stats->tx_packets++;
		break;
	case -ENOENT:
	case -ECONNRESET:
		/* urb has been unlinked */
		return;
	default:
		at76_dbg(DBG_URB, "%s - nonzero write bulk status received: %d",
			 __func__, urb->status);
		stats->tx_errors++;
		break;
	}

	spin_lock_irqsave(&priv->mgmt_spinlock, flags);
	mgmt_buf = priv->next_mgmt_bulk;
	priv->next_mgmt_bulk = NULL;
	spin_unlock_irqrestore(&priv->mgmt_spinlock, flags);

	if (!mgmt_buf) {
		netif_wake_queue(priv->netdev);
		return;
	}

	/* we don't copy the padding bytes, but add them
	   to the length */
	memcpy(priv->bulk_out_buffer, mgmt_buf,
	       le16_to_cpu(mgmt_buf->wlength) + AT76_TX_HDRLEN);
	usb_fill_bulk_urb(priv->write_urb, priv->udev, priv->tx_bulk_pipe,
			  priv->bulk_out_buffer,
			  le16_to_cpu(mgmt_buf->wlength) + mgmt_buf->padding +
			  AT76_TX_HDRLEN, at76_write_bulk_callback, priv);
	ret = usb_submit_urb(priv->write_urb, GFP_ATOMIC);
	if (ret)
		err("%s: %s error in tx submit urb: %d",
		    priv->netdev->name, __func__, ret);

	kfree(mgmt_buf);
}

/* Send a management frame on bulk-out.
   txbuf->wlength must be set (in LE format !) */
static int at76_send_mgmt_bulk(struct at76_priv *priv,
			       struct at76_tx_buffer *txbuf)
{
	unsigned long flags;
	int ret = 0;
	int urb_status;
	void *oldbuf = NULL;

	netif_carrier_off(priv->netdev);	/* stop netdev watchdog */
	netif_stop_queue(priv->netdev);	/* stop tx data packets */

	spin_lock_irqsave(&priv->mgmt_spinlock, flags);

	urb_status = priv->write_urb->status;
	if (urb_status == -EINPROGRESS) {
		oldbuf = priv->next_mgmt_bulk;	/* to kfree below */
		priv->next_mgmt_bulk = txbuf;
		txbuf = NULL;
	}
	spin_unlock_irqrestore(&priv->mgmt_spinlock, flags);

	if (oldbuf) {
		/* a data/mgmt tx is already pending in the URB -
		   if this is no error in some situations we must
		   implement a queue or silently modify the old msg */
		err("%s: %s removed pending mgmt buffer %s",
		    priv->netdev->name, __func__,
		    hex2str(priv->next_mgmt_bulk, 64));
		kfree(priv->next_mgmt_bulk);
	}

	if (!txbuf)
		return ret;

	txbuf->tx_rate = 0;
	txbuf->padding = at76_calc_padding(le16_to_cpu(txbuf->wlength));

	if (priv->next_mgmt_bulk)
		err("%s: %s URB status %d, but mgmt is pending",
		    priv->netdev->name, __func__, urb_status);

	at76_dbg(DBG_TX_MGMT,
		 "%s: tx mgmt: wlen %d tx_rate %d pad %d %s",
		 priv->netdev->name, le16_to_cpu(txbuf->wlength),
		 txbuf->tx_rate, txbuf->padding,
		 hex2str(txbuf->packet, le16_to_cpu(txbuf->wlength)));

	/* txbuf was not consumed above -> send mgmt msg immediately */
	memcpy(priv->bulk_out_buffer, txbuf,
	       le16_to_cpu(txbuf->wlength) + AT76_TX_HDRLEN);
	usb_fill_bulk_urb(priv->write_urb, priv->udev, priv->tx_bulk_pipe,
			  priv->bulk_out_buffer,
			  le16_to_cpu(txbuf->wlength) + txbuf->padding +
			  AT76_TX_HDRLEN, at76_write_bulk_callback, priv);
	ret = usb_submit_urb(priv->write_urb, GFP_ATOMIC);
	if (ret)
		err("%s: %s error in tx submit urb: %d",
		    priv->netdev->name, __func__, ret);

	kfree(txbuf);

	return ret;
}

/* Go to the next information element */
static inline void next_ie(struct ieee80211_info_element **ie)
{
	*ie = (struct ieee80211_info_element *)(&(*ie)->data[(*ie)->len]);
}

/* Challenge is the challenge string (in TLV format)
   we got with seq_nr 2 for shared secret authentication only and
   send in seq_nr 3 WEP encrypted to prove we have the correct WEP key;
   otherwise it is NULL */
static int at76_auth_req(struct at76_priv *priv, struct bss_info *bss,
			 int seq_nr, struct ieee80211_info_element *challenge)
{
	struct at76_tx_buffer *tx_buffer;
	struct ieee80211_hdr_3addr *mgmt;
	struct ieee80211_auth *req;
	int buf_len = (seq_nr != 3 ? AUTH_FRAME_SIZE :
		       AUTH_FRAME_SIZE + 1 + 1 + challenge->len);

	BUG_ON(bss == NULL);
	BUG_ON(seq_nr == 3 && challenge == NULL);
	tx_buffer = kmalloc(buf_len + MAX_PADDING_SIZE, GFP_ATOMIC);
	if (!tx_buffer)
		return -ENOMEM;

	req = (struct ieee80211_auth *)tx_buffer->packet;
	mgmt = &req->header;

	/* make wireless header */
	/* first auth msg is not encrypted, only the second (seq_nr == 3) */
	mgmt->frame_ctl =
	    cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_AUTH |
			(seq_nr == 3 ? IEEE80211_FCTL_PROTECTED : 0));

	mgmt->duration_id = cpu_to_le16(0x8000);
	memcpy(mgmt->addr1, bss->bssid, ETH_ALEN);
	memcpy(mgmt->addr2, priv->netdev->dev_addr, ETH_ALEN);
	memcpy(mgmt->addr3, bss->bssid, ETH_ALEN);
	mgmt->seq_ctl = cpu_to_le16(0);

	req->algorithm = cpu_to_le16(priv->auth_mode);
	req->transaction = cpu_to_le16(seq_nr);
	req->status = cpu_to_le16(0);

	if (seq_nr == 3)
		memcpy(req->info_element, challenge, 1 + 1 + challenge->len);

	/* init. at76_priv tx header */
	tx_buffer->wlength = cpu_to_le16(buf_len - AT76_TX_HDRLEN);
	at76_dbg(DBG_TX_MGMT, "%s: AuthReq bssid %s alg %d seq_nr %d",
		 priv->netdev->name, mac2str(mgmt->addr3),
		 le16_to_cpu(req->algorithm), le16_to_cpu(req->transaction));
	if (seq_nr == 3)
		at76_dbg(DBG_TX_MGMT, "%s: AuthReq challenge: %s ...",
			 priv->netdev->name, hex2str(req->info_element, 18));

	/* either send immediately (if no data tx is pending
	   or put it in pending list */
	return at76_send_mgmt_bulk(priv, tx_buffer);
}

static int at76_assoc_req(struct at76_priv *priv, struct bss_info *bss)
{
	struct at76_tx_buffer *tx_buffer;
	struct ieee80211_hdr_3addr *mgmt;
	struct ieee80211_assoc_request *req;
	struct ieee80211_info_element *tlv;
	char essid[IW_ESSID_MAX_SIZE + 1];
	int len;
	u16 capa;

	BUG_ON(bss == NULL);

	tx_buffer = kmalloc(ASSOCREQ_MAX_SIZE + MAX_PADDING_SIZE, GFP_ATOMIC);
	if (!tx_buffer)
		return -ENOMEM;

	req = (struct ieee80211_assoc_request *)tx_buffer->packet;
	mgmt = &req->header;
	tlv = req->info_element;

	/* make wireless header */
	mgmt->frame_ctl = cpu_to_le16(IEEE80211_FTYPE_MGMT |
				      IEEE80211_STYPE_ASSOC_REQ);

	mgmt->duration_id = cpu_to_le16(0x8000);
	memcpy(mgmt->addr1, bss->bssid, ETH_ALEN);
	memcpy(mgmt->addr2, priv->netdev->dev_addr, ETH_ALEN);
	memcpy(mgmt->addr3, bss->bssid, ETH_ALEN);
	mgmt->seq_ctl = cpu_to_le16(0);

	/* we must set the Privacy bit in the capabilities to assure an
	   Agere-based AP with optional WEP transmits encrypted frames
	   to us.  AP only set the Privacy bit in their capabilities
	   if WEP is mandatory in the BSS! */
	capa = bss->capa;
	if (priv->wep_enabled)
		capa |= WLAN_CAPABILITY_PRIVACY;
	if (priv->preamble_type != PREAMBLE_TYPE_LONG)
		capa |= WLAN_CAPABILITY_SHORT_PREAMBLE;
	req->capability = cpu_to_le16(capa);

	req->listen_interval = cpu_to_le16(2 * bss->beacon_interval);

	/* write TLV data elements */

	tlv->id = MFIE_TYPE_SSID;
	tlv->len = bss->ssid_len;
	memcpy(tlv->data, bss->ssid, bss->ssid_len);
	next_ie(&tlv);

	tlv->id = MFIE_TYPE_RATES;
	tlv->len = sizeof(hw_rates);
	memcpy(tlv->data, hw_rates, sizeof(hw_rates));
	next_ie(&tlv);		/* tlv points behind the supp_rates field */

	/* init. at76_priv tx header */
	tx_buffer->wlength = cpu_to_le16((u8 *)tlv - (u8 *)mgmt);

	tlv = req->info_element;
	len = min_t(int, IW_ESSID_MAX_SIZE, tlv->len);
	memcpy(essid, tlv->data, len);
	essid[len] = '\0';
	next_ie(&tlv);		/* points to IE of rates now */
	at76_dbg(DBG_TX_MGMT,
		 "%s: AssocReq bssid %s capa 0x%04x ssid %s rates %s",
		 priv->netdev->name, mac2str(mgmt->addr3),
		 le16_to_cpu(req->capability), essid,
		 hex2str(tlv->data, tlv->len));

	/* either send immediately (if no data tx is pending
	   or put it in pending list */
	return at76_send_mgmt_bulk(priv, tx_buffer);
}

/* We got to check the bss_list for old entries */
static void at76_bss_list_timeout(unsigned long par)
{
	struct at76_priv *priv = (struct at76_priv *)par;
	unsigned long flags;
	struct list_head *lptr, *nptr;
	struct bss_info *ptr;

	spin_lock_irqsave(&priv->bss_list_spinlock, flags);

	list_for_each_safe(lptr, nptr, &priv->bss_list) {

		ptr = list_entry(lptr, struct bss_info, list);

		if (ptr != priv->curr_bss
		    && time_after(jiffies, ptr->last_rx + BSS_LIST_TIMEOUT)) {
			at76_dbg(DBG_BSS_TABLE_RM,
				 "%s: bss_list: removing old BSS %s ch %d",
				 priv->netdev->name, mac2str(ptr->bssid),
				 ptr->channel);
			list_del(&ptr->list);
			kfree(ptr);
		}
	}
	spin_unlock_irqrestore(&priv->bss_list_spinlock, flags);
	/* restart the timer */
	mod_timer(&priv->bss_list_timer, jiffies + BSS_LIST_TIMEOUT);
}

static inline void at76_set_mac_state(struct at76_priv *priv,
				      enum mac_state mac_state)
{
	at76_dbg(DBG_MAC_STATE, "%s state: %s", priv->netdev->name,
		 mac_states[mac_state]);
	priv->mac_state = mac_state;
}

static void at76_dump_bss_table(struct at76_priv *priv)
{
	struct bss_info *ptr;
	unsigned long flags;
	struct list_head *lptr;

	spin_lock_irqsave(&priv->bss_list_spinlock, flags);

	pr_debug("%s BSS table (curr=%p):", priv->netdev->name, priv->curr_bss);

	list_for_each(lptr, &priv->bss_list) {
		ptr = list_entry(lptr, struct bss_info, list);
		pr_debug("0x%p: bssid %s channel %d ssid %s (%s)"
			 " capa 0x%04x rates %s rssi %d link %d noise %d",
			 ptr, mac2str(ptr->bssid),
			 ptr->channel,
			 ptr->ssid,
			 hex2str(ptr->ssid, ptr->ssid_len),
			 ptr->capa,
			 hex2str(ptr->rates, ptr->rates_len),
			 ptr->rssi, ptr->link_qual, ptr->noise_level);
	}
	spin_unlock_irqrestore(&priv->bss_list_spinlock, flags);
}

/* Called upon successful association to mark interface as connected */
static void at76_work_assoc_done(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      work_assoc_done);

	mutex_lock(&priv->mtx);

	WARN_ON(priv->mac_state != MAC_ASSOC);
	WARN_ON(priv->curr_bss == NULL);
	if (priv->mac_state != MAC_ASSOC || !priv->curr_bss)
		goto exit;

	if (priv->iw_mode == IW_MODE_INFRA) {
		if (priv->pm_mode != AT76_PM_OFF) {
			/* calculate the listen interval in units of
			   beacon intervals of the curr_bss */
			u32 pm_period_beacon = (priv->pm_period >> 10) /
			    priv->curr_bss->beacon_interval;

			pm_period_beacon = max(pm_period_beacon, 2u);
			pm_period_beacon = min(pm_period_beacon, 0xffffu);

			at76_dbg(DBG_PM,
				 "%s: pm_mode %d assoc id 0x%x listen int %d",
				 priv->netdev->name, priv->pm_mode,
				 priv->assoc_id, pm_period_beacon);

			at76_set_associd(priv, priv->assoc_id);
			at76_set_listen_interval(priv, (u16)pm_period_beacon);
#ifdef DEBUG
			at76_dump_mib_mac(priv);
			at76_dump_mib_mac_mgmt(priv);
#endif
		}
	}
	at76_set_pm_mode(priv);

	netif_carrier_on(priv->netdev);
	netif_wake_queue(priv->netdev);
	at76_set_mac_state(priv, MAC_CONNECTED);
	at76_iwevent_bss_connect(priv->netdev, priv->curr_bss->bssid);
	at76_dbg(DBG_PROGRESS, "%s: connected to BSSID %s",
		 priv->netdev->name, mac2str(priv->curr_bss->bssid));

exit:
	mutex_unlock(&priv->mtx);
}

static void at76_delete_device(struct at76_priv *priv)
{
	int i;

	at76_dbg(DBG_PROC_ENTRY, "%s: ENTER", __func__);

	/* The device is gone, don't bother turning it off */
	priv->device_unplugged = 1;

	tasklet_kill(&priv->rx_tasklet);

	if (priv->netdev_registered)
		unregister_netdev(priv->netdev);

	/* assuming we used keventd, it must quiesce too */
	flush_scheduled_work();

	if (priv->bulk_out_buffer != NULL)
		kfree(priv->bulk_out_buffer);

	if (priv->write_urb != NULL) {
		usb_kill_urb(priv->write_urb);
		usb_free_urb(priv->write_urb);
	}
	if (priv->read_urb != NULL) {
		usb_kill_urb(priv->read_urb);
		usb_free_urb(priv->read_urb);
	}

	at76_dbg(DBG_PROC_ENTRY, "%s: unlinked urbs", __func__);

	if (priv->rx_skb != NULL)
		kfree_skb(priv->rx_skb);

	at76_free_bss_list(priv);
	del_timer_sync(&priv->bss_list_timer);
	cancel_delayed_work(&priv->dwork_get_scan);
	cancel_delayed_work(&priv->dwork_beacon);
	cancel_delayed_work(&priv->dwork_auth);
	cancel_delayed_work(&priv->dwork_assoc);

	if (priv->mac_state == MAC_CONNECTED)
		at76_iwevent_bss_disconnect(priv->netdev);

	for (i = 0; i < NR_RX_DATA_BUF; i++)
		if (priv->rx_data[i].skb != NULL) {
			dev_kfree_skb(priv->rx_data[i].skb);
			priv->rx_data[i].skb = NULL;
		}
	usb_put_dev(priv->udev);

	at76_dbg(DBG_PROC_ENTRY, "%s: before freeing priv/netdev", __func__);
	free_netdev(priv->netdev);	/* priv is in netdev */

	at76_dbg(DBG_PROC_ENTRY, "%s: EXIT", __func__);
}

static int at76_alloc_urbs(struct at76_priv *priv,
			   struct usb_interface *interface)
{
	struct usb_endpoint_descriptor *endpoint;
	struct usb_device *udev = priv->udev;
	int i;
	int buffer_size;
	struct usb_host_interface *iface_desc;

	at76_dbg(DBG_PROC_ENTRY, "%s: ENTER", __func__);

	at76_dbg(DBG_URB, "%s: NumEndpoints %d ", __func__,
		 interface->altsetting[0].desc.bNumEndpoints);

	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		at76_dbg(DBG_URB, "%s: %d. endpoint: addr 0x%x attr 0x%x",
			 __func__,
			 i, endpoint->bEndpointAddress, endpoint->bmAttributes);

		if ((endpoint->bEndpointAddress & 0x80) &&
		    ((endpoint->bmAttributes & 3) == 0x02)) {
			/* we found a bulk in endpoint */

			priv->read_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!priv->read_urb) {
				err("No free urbs available");
				return -ENOMEM;
			}
			priv->rx_bulk_pipe =
			    usb_rcvbulkpipe(udev, endpoint->bEndpointAddress);
		}

		if (((endpoint->bEndpointAddress & 0x80) == 0x00) &&
		    ((endpoint->bmAttributes & 3) == 0x02)) {
			/* we found a bulk out endpoint */
			priv->write_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!priv->write_urb) {
				err("no free urbs available");
				return -ENOMEM;
			}
			buffer_size = sizeof(struct at76_tx_buffer) +
			    MAX_PADDING_SIZE;
			priv->tx_bulk_pipe =
			    usb_sndbulkpipe(udev, endpoint->bEndpointAddress);
			priv->bulk_out_buffer =
			    kmalloc(buffer_size, GFP_KERNEL);
			if (!priv->bulk_out_buffer) {
				err("couldn't allocate bulk_out_buffer");
				return -ENOMEM;
			}
		}
	}

	at76_dbg(DBG_PROC_ENTRY, "%s: EXIT", __func__);

	return 0;
}

/* We only store the new mac address in netdev struct,
   it gets set when the netdev is opened. */
static int at76_set_mac_address(struct net_device *netdev, void *addr)
{
	struct sockaddr *mac = addr;
	memcpy(netdev->dev_addr, mac->sa_data, ETH_ALEN);
	return 1;
}

static struct net_device_stats *at76_get_stats(struct net_device *netdev)
{
	struct at76_priv *priv = netdev_priv(netdev);
	return &priv->stats;
}

static struct iw_statistics *at76_get_wireless_stats(struct net_device *netdev)
{
	struct at76_priv *priv = netdev_priv(netdev);

	at76_dbg(DBG_IOCTL, "RETURN qual %d level %d noise %d updated %d",
		 priv->wstats.qual.qual, priv->wstats.qual.level,
		 priv->wstats.qual.noise, priv->wstats.qual.updated);

	return &priv->wstats;
}

static void at76_set_multicast(struct net_device *netdev)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int promisc;

	promisc = ((netdev->flags & IFF_PROMISC) != 0);
	if (promisc != priv->promisc) {
		/* This gets called in interrupt, must reschedule */
		priv->promisc = promisc;
		schedule_work(&priv->work_set_promisc);
	}
}

/*******************************************************************************
 * at76_priv implementations of iw_handler functions:
 */
static int at76_iw_handler_commit(struct net_device *netdev,
				  struct iw_request_info *info,
				  void *null, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	unsigned long flags;
	at76_dbg(DBG_IOCTL, "%s %s: restarting the device", netdev->name,
		 __func__);

	/* TODO: stop any pending tx bulk urb */
	if (priv->mac_state != MAC_INIT) {
		at76_set_mac_state(priv, MAC_INIT);
		/* stop pending management stuff */
		cancel_delayed_work(&priv->dwork_get_scan);
		cancel_delayed_work(&priv->dwork_beacon);
		cancel_delayed_work(&priv->dwork_auth);
		cancel_delayed_work(&priv->dwork_assoc);

		spin_lock_irqsave(&priv->mgmt_spinlock, flags);
		kfree(priv->next_mgmt_bulk);
		priv->next_mgmt_bulk = NULL;
		spin_unlock_irqrestore(&priv->mgmt_spinlock, flags);

		netif_carrier_off(priv->netdev);
		netif_stop_queue(priv->netdev);
	}

	/* Wait half second before the restart to process subsequent
	 * requests from the same iwconfig in a single restart */
	schedule_delayed_work(&priv->dwork_restart, HZ / 2);

	return 0;
}

static int at76_iw_handler_get_name(struct net_device *netdev,
				    struct iw_request_info *info,
				    char *name, char *extra)
{
	strcpy(name, "IEEE 802.11b");
	at76_dbg(DBG_IOCTL, "%s: SIOCGIWNAME - name %s", netdev->name, name);
	return 0;
}

static int at76_iw_handler_set_freq(struct net_device *netdev,
				    struct iw_request_info *info,
				    struct iw_freq *freq, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int chan = -1;
	int ret = -EIWCOMMIT;
	at76_dbg(DBG_IOCTL, "%s: SIOCSIWFREQ - freq.m %d freq.e %d",
		 netdev->name, freq->m, freq->e);

	if ((freq->e == 0) && (freq->m <= 1000))
		/* Setting by channel number */
		chan = freq->m;
	else {
		/* Setting by frequency - search the table */
		int mult = 1;
		int i;

		for (i = 0; i < (6 - freq->e); i++)
			mult *= 10;

		for (i = 0; i < NUM_CHANNELS; i++) {
			if (freq->m == (channel_frequency[i] * mult))
				chan = i + 1;
		}
	}

	if (chan < 1 || !priv->domain)
		/* non-positive channels are invalid
		 * we need a domain info to set the channel
		 * either that or an invalid frequency was
		 * provided by the user */
		ret = -EINVAL;
	else if (!priv->international_roaming) {
		if (!(priv->domain->channel_map & (1 << (chan - 1)))) {
			printk(KERN_INFO
			       "%s: channel %d not allowed for domain %s "
			       "(and international_roaming is OFF)\n",
			       priv->netdev->name, chan, priv->domain->name);
			ret = -EINVAL;
		}
	}

	if (ret == -EIWCOMMIT) {
		priv->channel = chan;
		at76_dbg(DBG_IOCTL, "%s: SIOCSIWFREQ - ch %d", netdev->name,
			 chan);
	}

	return ret;
}

static int at76_iw_handler_get_freq(struct net_device *netdev,
				    struct iw_request_info *info,
				    struct iw_freq *freq, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	freq->m = priv->channel;
	freq->e = 0;

	if (priv->channel)
		at76_dbg(DBG_IOCTL, "%s: SIOCGIWFREQ - freq %ld x 10e%d",
			 netdev->name, channel_frequency[priv->channel - 1], 6);

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWFREQ - ch %d", netdev->name,
		 priv->channel);

	return 0;
}

static int at76_iw_handler_set_mode(struct net_device *netdev,
				    struct iw_request_info *info,
				    __u32 *mode, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWMODE - %d", netdev->name, *mode);

	if ((*mode != IW_MODE_ADHOC) && (*mode != IW_MODE_INFRA) &&
	    (*mode != IW_MODE_MONITOR))
		return -EINVAL;

	priv->iw_mode = *mode;
	if (priv->iw_mode != IW_MODE_INFRA)
		priv->pm_mode = AT76_PM_OFF;

	return -EIWCOMMIT;
}

static int at76_iw_handler_get_mode(struct net_device *netdev,
				    struct iw_request_info *info,
				    __u32 *mode, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	*mode = priv->iw_mode;

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWMODE - %d", netdev->name, *mode);

	return 0;
}

static int at76_iw_handler_get_range(struct net_device *netdev,
				     struct iw_request_info *info,
				     struct iw_point *data, char *extra)
{
	/* inspired by atmel.c */
	struct at76_priv *priv = netdev_priv(netdev);
	struct iw_range *range = (struct iw_range *)extra;
	int i;

	data->length = sizeof(struct iw_range);
	memset(range, 0, sizeof(struct iw_range));

	/* TODO: range->throughput = xxxxxx; */

	range->min_nwid = 0x0000;
	range->max_nwid = 0x0000;

	/* this driver doesn't maintain sensitivity information */
	range->sensitivity = 0;

	range->max_qual.qual = 100;
	range->max_qual.level = 100;
	range->max_qual.noise = 0;
	range->max_qual.updated = IW_QUAL_NOISE_INVALID;

	range->avg_qual.qual = 50;
	range->avg_qual.level = 50;
	range->avg_qual.noise = 0;
	range->avg_qual.updated = IW_QUAL_NOISE_INVALID;

	range->bitrate[0] = 1000000;
	range->bitrate[1] = 2000000;
	range->bitrate[2] = 5500000;
	range->bitrate[3] = 11000000;
	range->num_bitrates = 4;

	range->min_rts = 0;
	range->max_rts = MAX_RTS_THRESHOLD;

	range->min_frag = MIN_FRAG_THRESHOLD;
	range->max_frag = MAX_FRAG_THRESHOLD;

	range->pmp_flags = IW_POWER_PERIOD;
	range->pmt_flags = IW_POWER_ON;
	range->pm_capa = IW_POWER_PERIOD | IW_POWER_ALL_R;

	range->encoding_size[0] = WEP_SMALL_KEY_LEN;
	range->encoding_size[1] = WEP_LARGE_KEY_LEN;
	range->num_encoding_sizes = 2;
	range->max_encoding_tokens = WEP_KEYS;

	/* both WL-240U and Linksys WUSB11 v2.6 specify 15 dBm as output power
	   - take this for all (ignore antenna gains) */
	range->txpower[0] = 15;
	range->num_txpower = 1;
	range->txpower_capa = IW_TXPOW_DBM;

	range->we_version_source = WIRELESS_EXT;
	range->we_version_compiled = WIRELESS_EXT;

	/* same as the values used in atmel.c */
	range->retry_capa = IW_RETRY_LIMIT;
	range->retry_flags = IW_RETRY_LIMIT;
	range->r_time_flags = 0;
	range->min_retry = 1;
	range->max_retry = 255;

	range->num_channels = NUM_CHANNELS;
	range->num_frequency = 0;

	for (i = 0; i < NUM_CHANNELS; i++) {
		/* test if channel map bit is raised */
		if (priv->domain->channel_map & (0x1 << i)) {
			range->num_frequency += 1;

			range->freq[i].i = i + 1;
			range->freq[i].m = channel_frequency[i] * 100000;
			range->freq[i].e = 1;	/* freq * 10^1 */
		}
	}

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWRANGE", netdev->name);

	return 0;
}

static int at76_iw_handler_set_spy(struct net_device *netdev,
				   struct iw_request_info *info,
				   struct iw_point *data, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret = 0;

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWSPY - number of addresses %d",
		 netdev->name, data->length);

	spin_lock_bh(&priv->spy_spinlock);
	ret = iw_handler_set_spy(priv->netdev, info, (union iwreq_data *)data,
				 extra);
	spin_unlock_bh(&priv->spy_spinlock);

	return ret;
}

static int at76_iw_handler_get_spy(struct net_device *netdev,
				   struct iw_request_info *info,
				   struct iw_point *data, char *extra)
{

	struct at76_priv *priv = netdev_priv(netdev);
	int ret = 0;

	spin_lock_bh(&priv->spy_spinlock);
	ret = iw_handler_get_spy(priv->netdev, info,
				 (union iwreq_data *)data, extra);
	spin_unlock_bh(&priv->spy_spinlock);

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWSPY - number of addresses %d",
		 netdev->name, data->length);

	return ret;
}

static int at76_iw_handler_set_thrspy(struct net_device *netdev,
				      struct iw_request_info *info,
				      struct iw_point *data, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret;

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWTHRSPY - number of addresses %d)",
		 netdev->name, data->length);

	spin_lock_bh(&priv->spy_spinlock);
	ret = iw_handler_set_thrspy(netdev, info, (union iwreq_data *)data,
				    extra);
	spin_unlock_bh(&priv->spy_spinlock);

	return ret;
}

static int at76_iw_handler_get_thrspy(struct net_device *netdev,
				      struct iw_request_info *info,
				      struct iw_point *data, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret;

	spin_lock_bh(&priv->spy_spinlock);
	ret = iw_handler_get_thrspy(netdev, info, (union iwreq_data *)data,
				    extra);
	spin_unlock_bh(&priv->spy_spinlock);

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWTHRSPY - number of addresses %d)",
		 netdev->name, data->length);

	return ret;
}

static int at76_iw_handler_set_wap(struct net_device *netdev,
				   struct iw_request_info *info,
				   struct sockaddr *ap_addr, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWAP - wap/bssid %s", netdev->name,
		 mac2str(ap_addr->sa_data));

	/* if the incoming address == ff:ff:ff:ff:ff:ff, the user has
	   chosen any or auto AP preference */
	if (is_broadcast_ether_addr(ap_addr->sa_data)
	    || is_zero_ether_addr(ap_addr->sa_data))
		priv->wanted_bssid_valid = 0;
	else {
		/* user wants to set a preferred AP address */
		priv->wanted_bssid_valid = 1;
		memcpy(priv->wanted_bssid, ap_addr->sa_data, ETH_ALEN);
	}

	return -EIWCOMMIT;
}

static int at76_iw_handler_get_wap(struct net_device *netdev,
				   struct iw_request_info *info,
				   struct sockaddr *ap_addr, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	ap_addr->sa_family = ARPHRD_ETHER;
	memcpy(ap_addr->sa_data, priv->bssid, ETH_ALEN);

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWAP - wap/bssid %s", netdev->name,
		 mac2str(ap_addr->sa_data));

	return 0;
}

static int at76_iw_handler_set_scan(struct net_device *netdev,
				    struct iw_request_info *info,
				    union iwreq_data *wrqu, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	unsigned long flags;
	int ret = 0;
	struct iw_scan_req *req = NULL;

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWSCAN", netdev->name);

	if (!netif_running(netdev))
		return -ENETDOWN;

	/* jal: we don't allow "iwlist ethX scan" while we are
	   in monitor mode */
	if (priv->iw_mode == IW_MODE_MONITOR)
		return -EBUSY;

	/* Discard old scan results */
	if ((jiffies - priv->last_scan) > (20 * HZ))
		priv->scan_state = SCAN_IDLE;
	priv->last_scan = jiffies;

	/* Initiate a scan command */
	if (priv->scan_state == SCAN_IN_PROGRESS)
		return -EBUSY;

	priv->scan_state = SCAN_IN_PROGRESS;

	/* stop pending management stuff */
	cancel_delayed_work(&priv->dwork_get_scan);
	cancel_delayed_work(&priv->dwork_beacon);
	cancel_delayed_work(&priv->dwork_auth);
	cancel_delayed_work(&priv->dwork_assoc);

	spin_lock_irqsave(&priv->mgmt_spinlock, flags);
	kfree(priv->next_mgmt_bulk);
	priv->next_mgmt_bulk = NULL;
	spin_unlock_irqrestore(&priv->mgmt_spinlock, flags);

	if (netif_running(priv->netdev)) {
		/* pause network activity */
		netif_carrier_off(priv->netdev);
		netif_stop_queue(priv->netdev);
	}
	/* Try to do passive or active scan if WE asks as. */
	if (wrqu->data.length
	    && wrqu->data.length == sizeof(struct iw_scan_req)) {
		req = (struct iw_scan_req *)extra;

		if (req->scan_type == IW_SCAN_TYPE_PASSIVE)
			priv->scan_mode = SCAN_TYPE_PASSIVE;
		else if (req->scan_type == IW_SCAN_TYPE_ACTIVE)
			priv->scan_mode = SCAN_TYPE_ACTIVE;

		/* Sanity check values? */
		if (req->min_channel_time > 0)
			priv->scan_min_time = req->min_channel_time;

		if (req->max_channel_time > 0)
			priv->scan_max_time = req->max_channel_time;
	}

	/* change to scanning state */
	at76_set_mac_state(priv, MAC_SCANNING);
	schedule_work(&priv->work_start_scan);

	return ret;
}

static int at76_iw_handler_get_scan(struct net_device *netdev,
				    struct iw_request_info *info,
				    struct iw_point *data, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	unsigned long flags;
	struct list_head *lptr, *nptr;
	struct bss_info *curr_bss;
	struct iw_event *iwe = kmalloc(sizeof(struct iw_event), GFP_KERNEL);
	char *curr_val, *curr_pos = extra;
	int i;

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWSCAN", netdev->name);

	if (!iwe)
		return -ENOMEM;

	if (priv->scan_state != SCAN_COMPLETED)
		/* scan not yet finished */
		return -EAGAIN;

	spin_lock_irqsave(&priv->bss_list_spinlock, flags);

	list_for_each_safe(lptr, nptr, &priv->bss_list) {
		curr_bss = list_entry(lptr, struct bss_info, list);

		iwe->cmd = SIOCGIWAP;
		iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
		memcpy(iwe->u.ap_addr.sa_data, curr_bss->bssid, 6);
		curr_pos = iwe_stream_add_event(curr_pos,
						extra + IW_SCAN_MAX_DATA, iwe,
						IW_EV_ADDR_LEN);

		iwe->u.data.length = curr_bss->ssid_len;
		iwe->cmd = SIOCGIWESSID;
		iwe->u.data.flags = 1;

		curr_pos = iwe_stream_add_point(curr_pos,
						extra + IW_SCAN_MAX_DATA, iwe,
						curr_bss->ssid);

		iwe->cmd = SIOCGIWMODE;
		iwe->u.mode = (curr_bss->capa & WLAN_CAPABILITY_IBSS) ?
		    IW_MODE_ADHOC :
		    (curr_bss->capa & WLAN_CAPABILITY_ESS) ?
		    IW_MODE_MASTER : IW_MODE_AUTO;
		/* IW_MODE_AUTO = 0 which I thought is
		 * the most logical value to return in this case */
		curr_pos = iwe_stream_add_event(curr_pos,
						extra + IW_SCAN_MAX_DATA, iwe,
						IW_EV_UINT_LEN);

		iwe->cmd = SIOCGIWFREQ;
		iwe->u.freq.m = curr_bss->channel;
		iwe->u.freq.e = 0;
		curr_pos = iwe_stream_add_event(curr_pos,
						extra + IW_SCAN_MAX_DATA, iwe,
						IW_EV_FREQ_LEN);

		iwe->cmd = SIOCGIWENCODE;
		if (curr_bss->capa & WLAN_CAPABILITY_PRIVACY)
			iwe->u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
		else
			iwe->u.data.flags = IW_ENCODE_DISABLED;

		iwe->u.data.length = 0;
		curr_pos = iwe_stream_add_point(curr_pos,
						extra + IW_SCAN_MAX_DATA, iwe,
						NULL);

		/* Add quality statistics */
		iwe->cmd = IWEVQUAL;
		iwe->u.qual.noise = 0;
		iwe->u.qual.updated =
		    IW_QUAL_NOISE_INVALID | IW_QUAL_LEVEL_UPDATED;
		iwe->u.qual.level = (curr_bss->rssi * 100 / 42);
		if (iwe->u.qual.level > 100)
			iwe->u.qual.level = 100;
		if ((priv->board_type == BOARD_503_ISL3861) ||
		    (priv->board_type == BOARD_503_ISL3863))
			iwe->u.qual.qual = curr_bss->link_qual;
		else {
			iwe->u.qual.qual = 0;
			iwe->u.qual.updated |= IW_QUAL_QUAL_INVALID;
		}
		/* Add new value to event */
		curr_pos = iwe_stream_add_event(curr_pos,
						extra + IW_SCAN_MAX_DATA, iwe,
						IW_EV_QUAL_LEN);

		/* Rate: stuffing multiple values in a single event requires
		 * a bit more of magic - Jean II */
		curr_val = curr_pos + IW_EV_LCP_LEN;

		iwe->cmd = SIOCGIWRATE;
		/* Those two flags are ignored... */
		iwe->u.bitrate.fixed = 0;
		iwe->u.bitrate.disabled = 0;
		/* Max 8 values */
		for (i = 0; i < curr_bss->rates_len; i++) {
			/* Bit rate given in 500 kb/s units (+ 0x80) */
			iwe->u.bitrate.value =
			    ((curr_bss->rates[i] & 0x7f) * 500000);
			/* Add new value to event */
			curr_val = iwe_stream_add_value(curr_pos, curr_val,
							extra +
							IW_SCAN_MAX_DATA, iwe,
							IW_EV_PARAM_LEN);
		}

		/* Check if we added any event */
		if ((curr_val - curr_pos) > IW_EV_LCP_LEN)
			curr_pos = curr_val;

		/* more information may be sent back using IWECUSTOM */

	}

	spin_unlock_irqrestore(&priv->bss_list_spinlock, flags);

	data->length = (curr_pos - extra);
	data->flags = 0;

	kfree(iwe);
	return 0;
}

static int at76_iw_handler_set_essid(struct net_device *netdev,
				     struct iw_request_info *info,
				     struct iw_point *data, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWESSID - %s", netdev->name, extra);

	if (data->flags) {
		memcpy(priv->essid, extra, data->length);
		priv->essid_size = data->length;
	} else
		priv->essid_size = 0;	/* Use any SSID */

	return -EIWCOMMIT;
}

static int at76_iw_handler_get_essid(struct net_device *netdev,
				     struct iw_request_info *info,
				     struct iw_point *data, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	if (priv->essid_size) {
		/* not the ANY ssid in priv->essid */
		data->flags = 1;
		data->length = priv->essid_size;
		memcpy(extra, priv->essid, data->length);
		extra[data->length] = '\0';
		data->length += 1;
	} else {
		/* the ANY ssid was specified */
		if (priv->mac_state == MAC_CONNECTED && priv->curr_bss) {
			/* report the SSID we have found */
			data->flags = 1;
			data->length = priv->curr_bss->ssid_len;
			memcpy(extra, priv->curr_bss->ssid, data->length);
			extra[priv->curr_bss->ssid_len] = '\0';
			data->length += 1;
		} else {
			/* report ANY back */
			data->flags = 0;
			data->length = 0;
		}
	}

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWESSID - %s", netdev->name, extra);

	return 0;
}

static int at76_iw_handler_set_rate(struct net_device *netdev,
				    struct iw_request_info *info,
				    struct iw_param *bitrate, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret = -EIWCOMMIT;

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWRATE - %d", netdev->name,
		 bitrate->value);

	switch (bitrate->value) {
	case -1:
		priv->txrate = TX_RATE_AUTO;
		break;		/* auto rate */
	case 1000000:
		priv->txrate = TX_RATE_1MBIT;
		break;
	case 2000000:
		priv->txrate = TX_RATE_2MBIT;
		break;
	case 5500000:
		priv->txrate = TX_RATE_5_5MBIT;
		break;
	case 11000000:
		priv->txrate = TX_RATE_11MBIT;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int at76_iw_handler_get_rate(struct net_device *netdev,
				    struct iw_request_info *info,
				    struct iw_param *bitrate, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret = 0;

	switch (priv->txrate) {
		/* return max rate if RATE_AUTO */
	case TX_RATE_AUTO:
		bitrate->value = 11000000;
		break;
	case TX_RATE_1MBIT:
		bitrate->value = 1000000;
		break;
	case TX_RATE_2MBIT:
		bitrate->value = 2000000;
		break;
	case TX_RATE_5_5MBIT:
		bitrate->value = 5500000;
		break;
	case TX_RATE_11MBIT:
		bitrate->value = 11000000;
		break;
	default:
		ret = -EINVAL;
	}

	bitrate->fixed = (priv->txrate != TX_RATE_AUTO);
	bitrate->disabled = 0;

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWRATE - %d", netdev->name,
		 bitrate->value);

	return ret;
}

static int at76_iw_handler_set_rts(struct net_device *netdev,
				   struct iw_request_info *info,
				   struct iw_param *rts, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret = -EIWCOMMIT;
	int rthr = rts->value;

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWRTS - value %d disabled %s",
		 netdev->name, rts->value, (rts->disabled) ? "true" : "false");

	if (rts->disabled)
		rthr = MAX_RTS_THRESHOLD;

	if ((rthr < 0) || (rthr > MAX_RTS_THRESHOLD))
		ret = -EINVAL;
	else
		priv->rts_threshold = rthr;

	return ret;
}

static int at76_iw_handler_get_rts(struct net_device *netdev,
				   struct iw_request_info *info,
				   struct iw_param *rts, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	rts->value = priv->rts_threshold;
	rts->disabled = (rts->value >= MAX_RTS_THRESHOLD);
	rts->fixed = 1;

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWRTS - value %d disabled %s",
		 netdev->name, rts->value, (rts->disabled) ? "true" : "false");

	return 0;
}

static int at76_iw_handler_set_frag(struct net_device *netdev,
				    struct iw_request_info *info,
				    struct iw_param *frag, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret = -EIWCOMMIT;
	int fthr = frag->value;

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWFRAG - value %d, disabled %s",
		 netdev->name, frag->value,
		 (frag->disabled) ? "true" : "false");

	if (frag->disabled)
		fthr = MAX_FRAG_THRESHOLD;

	if ((fthr < MIN_FRAG_THRESHOLD) || (fthr > MAX_FRAG_THRESHOLD))
		ret = -EINVAL;
	else
		priv->frag_threshold = fthr & ~0x1;	/* get an even value */

	return ret;
}

static int at76_iw_handler_get_frag(struct net_device *netdev,
				    struct iw_request_info *info,
				    struct iw_param *frag, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	frag->value = priv->frag_threshold;
	frag->disabled = (frag->value >= MAX_FRAG_THRESHOLD);
	frag->fixed = 1;

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWFRAG - value %d, disabled %s",
		 netdev->name, frag->value,
		 (frag->disabled) ? "true" : "false");

	return 0;
}

static int at76_iw_handler_get_txpow(struct net_device *netdev,
				     struct iw_request_info *info,
				     struct iw_param *power, char *extra)
{
	power->value = 15;
	power->fixed = 1;	/* No power control */
	power->disabled = 0;
	power->flags = IW_TXPOW_DBM;

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWTXPOW - txpow %d dBm", netdev->name,
		 power->value);

	return 0;
}

/* jal: short retry is handled by the firmware (at least 0.90.x),
   while long retry is not (?) */
static int at76_iw_handler_set_retry(struct net_device *netdev,
				     struct iw_request_info *info,
				     struct iw_param *retry, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret = -EIWCOMMIT;

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWRETRY disabled %d flags 0x%x val %d",
		 netdev->name, retry->disabled, retry->flags, retry->value);

	if (!retry->disabled && (retry->flags & IW_RETRY_LIMIT)) {
		if ((retry->flags & IW_RETRY_MIN) ||
		    !(retry->flags & IW_RETRY_MAX))
			priv->short_retry_limit = retry->value;
		else
			ret = -EINVAL;
	} else
		ret = -EINVAL;

	return ret;
}

/* Adapted (ripped) from atmel.c */
static int at76_iw_handler_get_retry(struct net_device *netdev,
				     struct iw_request_info *info,
				     struct iw_param *retry, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWRETRY", netdev->name);

	retry->disabled = 0;	/* Can't be disabled */
	retry->flags = IW_RETRY_LIMIT;
	retry->value = priv->short_retry_limit;

	return 0;
}

static int at76_iw_handler_set_encode(struct net_device *netdev,
				      struct iw_request_info *info,
				      struct iw_point *encoding, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int index = (encoding->flags & IW_ENCODE_INDEX) - 1;
	int len = encoding->length;

	at76_dbg(DBG_IOCTL, "%s: SIOCSIWENCODE - enc.flags %08x "
		 "pointer %p len %d", netdev->name, encoding->flags,
		 encoding->pointer, encoding->length);
	at76_dbg(DBG_IOCTL,
		 "%s: SIOCSIWENCODE - old wepstate: enabled %s key_id %d "
		 "auth_mode %s", netdev->name,
		 (priv->wep_enabled) ? "true" : "false", priv->wep_key_id,
		 (priv->auth_mode ==
		  WLAN_AUTH_SHARED_KEY) ? "restricted" : "open");

	/* take the old default key if index is invalid */
	if ((index < 0) || (index >= WEP_KEYS))
		index = priv->wep_key_id;

	if (len > 0) {
		if (len > WEP_LARGE_KEY_LEN)
			len = WEP_LARGE_KEY_LEN;

		memset(priv->wep_keys[index], 0, WEP_KEY_LEN);
		memcpy(priv->wep_keys[index], extra, len);
		priv->wep_keys_len[index] = (len <= WEP_SMALL_KEY_LEN) ?
		    WEP_SMALL_KEY_LEN : WEP_LARGE_KEY_LEN;
		priv->wep_enabled = 1;
	}

	priv->wep_key_id = index;
	priv->wep_enabled = ((encoding->flags & IW_ENCODE_DISABLED) == 0);

	if (encoding->flags & IW_ENCODE_RESTRICTED)
		priv->auth_mode = WLAN_AUTH_SHARED_KEY;
	if (encoding->flags & IW_ENCODE_OPEN)
		priv->auth_mode = WLAN_AUTH_OPEN;

	at76_dbg(DBG_IOCTL,
		 "%s: SIOCSIWENCODE - new wepstate: enabled %s key_id %d "
		 "key_len %d auth_mode %s", netdev->name,
		 (priv->wep_enabled) ? "true" : "false", priv->wep_key_id + 1,
		 priv->wep_keys_len[priv->wep_key_id],
		 (priv->auth_mode ==
		  WLAN_AUTH_SHARED_KEY) ? "restricted" : "open");

	return -EIWCOMMIT;
}

static int at76_iw_handler_get_encode(struct net_device *netdev,
				      struct iw_request_info *info,
				      struct iw_point *encoding, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int index = (encoding->flags & IW_ENCODE_INDEX) - 1;

	if ((index < 0) || (index >= WEP_KEYS))
		index = priv->wep_key_id;

	encoding->flags =
	    (priv->auth_mode == WLAN_AUTH_SHARED_KEY) ?
	    IW_ENCODE_RESTRICTED : IW_ENCODE_OPEN;

	if (!priv->wep_enabled)
		encoding->flags |= IW_ENCODE_DISABLED;

	if (encoding->pointer) {
		encoding->length = priv->wep_keys_len[index];

		memcpy(extra, priv->wep_keys[index], priv->wep_keys_len[index]);

		encoding->flags |= (index + 1);
	}

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWENCODE - enc.flags %08x "
		 "pointer %p len %d", netdev->name, encoding->flags,
		 encoding->pointer, encoding->length);
	at76_dbg(DBG_IOCTL,
		 "%s: SIOCGIWENCODE - wepstate: enabled %s key_id %d "
		 "key_len %d auth_mode %s", netdev->name,
		 (priv->wep_enabled) ? "true" : "false", priv->wep_key_id + 1,
		 priv->wep_keys_len[priv->wep_key_id],
		 (priv->auth_mode ==
		  WLAN_AUTH_SHARED_KEY) ? "restricted" : "open");

	return 0;
}

static int at76_iw_handler_set_power(struct net_device *netdev,
				     struct iw_request_info *info,
				     struct iw_param *prq, char *extra)
{
	int err = -EIWCOMMIT;
	struct at76_priv *priv = netdev_priv(netdev);

	at76_dbg(DBG_IOCTL,
		 "%s: SIOCSIWPOWER - disabled %s flags 0x%x value 0x%x",
		 netdev->name, (prq->disabled) ? "true" : "false", prq->flags,
		 prq->value);

	if (prq->disabled)
		priv->pm_mode = AT76_PM_OFF;
	else {
		switch (prq->flags & IW_POWER_MODE) {
		case IW_POWER_ALL_R:
		case IW_POWER_ON:
			break;
		default:
			err = -EINVAL;
			goto exit;
		}
		if (prq->flags & IW_POWER_PERIOD)
			priv->pm_period = prq->value;

		if (prq->flags & IW_POWER_TIMEOUT) {
			err = -EINVAL;
			goto exit;
		}
		priv->pm_mode = AT76_PM_ON;
	}
exit:
	return err;
}

static int at76_iw_handler_get_power(struct net_device *netdev,
				     struct iw_request_info *info,
				     struct iw_param *power, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	power->disabled = (priv->pm_mode == AT76_PM_OFF);
	if (!power->disabled) {
		power->flags = IW_POWER_PERIOD | IW_POWER_ALL_R;
		power->value = priv->pm_period;
	}

	at76_dbg(DBG_IOCTL, "%s: SIOCGIWPOWER - %s flags 0x%x value 0x%x",
		 netdev->name, power->disabled ? "disabled" : "enabled",
		 power->flags, power->value);

	return 0;
}

/*******************************************************************************
 * Private IOCTLS
 */
static int at76_iw_set_short_preamble(struct net_device *netdev,
				      struct iw_request_info *info, char *name,
				      char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int val = *((int *)name);
	int ret = -EIWCOMMIT;

	at76_dbg(DBG_IOCTL, "%s: AT76_SET_SHORT_PREAMBLE, %d",
		 netdev->name, val);

	if (val < PREAMBLE_TYPE_LONG || val > PREAMBLE_TYPE_AUTO)
		ret = -EINVAL;
	else
		priv->preamble_type = val;

	return ret;
}

static int at76_iw_get_short_preamble(struct net_device *netdev,
				      struct iw_request_info *info,
				      union iwreq_data *wrqu, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);

	snprintf(wrqu->name, sizeof(wrqu->name), "%s (%d)",
		 preambles[priv->preamble_type], priv->preamble_type);
	return 0;
}

static int at76_iw_set_debug(struct net_device *netdev,
			     struct iw_request_info *info,
			     struct iw_point *data, char *extra)
{
	char *ptr;
	u32 val;

	if (data->length > 0) {
		val = simple_strtol(extra, &ptr, 0);

		if (ptr == extra)
			val = DBG_DEFAULTS;

		dbg("%s: AT76_SET_DEBUG input %d: %s -> 0x%x",
		    netdev->name, data->length, extra, val);
	} else
		val = DBG_DEFAULTS;

	dbg("%s: AT76_SET_DEBUG, old 0x%x, new 0x%x",
	    netdev->name, at76_debug, val);

	/* jal: some more output to pin down lockups */
	dbg("%s: netif running %d queue_stopped %d carrier_ok %d",
	    netdev->name,
	    netif_running(netdev),
	    netif_queue_stopped(netdev), netif_carrier_ok(netdev));

	at76_debug = val;

	return 0;
}

static int at76_iw_get_debug(struct net_device *netdev,
			     struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	snprintf(wrqu->name, sizeof(wrqu->name), "0x%08x", at76_debug);
	return 0;
}

static int at76_iw_set_powersave_mode(struct net_device *netdev,
				      struct iw_request_info *info, char *name,
				      char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int val = *((int *)name);
	int ret = -EIWCOMMIT;

	at76_dbg(DBG_IOCTL, "%s: AT76_SET_POWERSAVE_MODE, %d (%s)",
		 netdev->name, val,
		 val == AT76_PM_OFF ? "active" : val == AT76_PM_ON ? "save" :
		 val == AT76_PM_SMART ? "smart save" : "<invalid>");
	if (val < AT76_PM_OFF || val > AT76_PM_SMART)
		ret = -EINVAL;
	else
		priv->pm_mode = val;

	return ret;
}

static int at76_iw_get_powersave_mode(struct net_device *netdev,
				      struct iw_request_info *info,
				      union iwreq_data *wrqu, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int *param = (int *)extra;

	param[0] = priv->pm_mode;
	return 0;
}

static int at76_iw_set_scan_times(struct net_device *netdev,
				  struct iw_request_info *info, char *name,
				  char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int mint = *((int *)name);
	int maxt = *((int *)name + 1);
	int ret = -EIWCOMMIT;

	at76_dbg(DBG_IOCTL, "%s: AT76_SET_SCAN_TIMES - min %d max %d",
		 netdev->name, mint, maxt);
	if (mint <= 0 || maxt <= 0 || mint > maxt)
		ret = -EINVAL;
	else {
		priv->scan_min_time = mint;
		priv->scan_max_time = maxt;
	}

	return ret;
}

static int at76_iw_get_scan_times(struct net_device *netdev,
				  struct iw_request_info *info,
				  union iwreq_data *wrqu, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int *param = (int *)extra;

	param[0] = priv->scan_min_time;
	param[1] = priv->scan_max_time;
	return 0;
}

static int at76_iw_set_scan_mode(struct net_device *netdev,
				 struct iw_request_info *info, char *name,
				 char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int val = *((int *)name);
	int ret = -EIWCOMMIT;

	at76_dbg(DBG_IOCTL, "%s: AT76_SET_SCAN_MODE - mode %s",
		 netdev->name, (val = SCAN_TYPE_ACTIVE) ? "active" :
		 (val = SCAN_TYPE_PASSIVE) ? "passive" : "<invalid>");

	if (val != SCAN_TYPE_ACTIVE && val != SCAN_TYPE_PASSIVE)
		ret = -EINVAL;
	else
		priv->scan_mode = val;

	return ret;
}

static int at76_iw_get_scan_mode(struct net_device *netdev,
				 struct iw_request_info *info,
				 union iwreq_data *wrqu, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int *param = (int *)extra;

	param[0] = priv->scan_mode;
	return 0;
}

static int at76_set_iroaming(struct at76_priv *priv, int onoff)
{
	int ret = 0;

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC_MGMT;
	priv->mib_buf.size = 1;
	priv->mib_buf.index =
	    offsetof(struct mib_mac_mgmt, multi_domain_capability_enabled);
	priv->mib_buf.data[0] = onoff;
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (intl_roaming_enable) failed: %d",
		    priv->netdev->name, ret);

	return ret;
}

static int at76_iw_set_intl_roaming(struct net_device *netdev,
				    struct iw_request_info *info, char *name,
				    char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int val = *((int *)name);
	int ret = -EIWCOMMIT;

	at76_dbg(DBG_IOCTL, "%s: AT76_SET_INTL_ROAMING - mode %s",
		 netdev->name, (val == IR_OFF) ? "off" :
		 (val == IR_ON) ? "on" : "<invalid>");

	if (val != IR_OFF && val != IR_ON)
		ret = -EINVAL;
	else {
		if (priv->international_roaming != val) {
			priv->international_roaming = val;
			at76_set_iroaming(priv, val);
		}
	}

	return ret;
}

static int at76_iw_get_intl_roaming(struct net_device *netdev,
				    struct iw_request_info *info,
				    union iwreq_data *wrqu, char *extra)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int *param = (int *)extra;

	param[0] = priv->international_roaming;
	return 0;
}

#define AT76_SET_HANDLER(h, f) [h - SIOCIWFIRST] = (iw_handler) f

/* Standard wireless handlers */
static const iw_handler at76_handlers[] = {
	AT76_SET_HANDLER(SIOCSIWCOMMIT, at76_iw_handler_commit),
	AT76_SET_HANDLER(SIOCGIWNAME, at76_iw_handler_get_name),
	AT76_SET_HANDLER(SIOCSIWFREQ, at76_iw_handler_set_freq),
	AT76_SET_HANDLER(SIOCGIWFREQ, at76_iw_handler_get_freq),
	AT76_SET_HANDLER(SIOCSIWMODE, at76_iw_handler_set_mode),
	AT76_SET_HANDLER(SIOCGIWMODE, at76_iw_handler_get_mode),
	AT76_SET_HANDLER(SIOCGIWRANGE, at76_iw_handler_get_range),
	AT76_SET_HANDLER(SIOCSIWSPY, at76_iw_handler_set_spy),
	AT76_SET_HANDLER(SIOCGIWSPY, at76_iw_handler_get_spy),
	AT76_SET_HANDLER(SIOCSIWTHRSPY, at76_iw_handler_set_thrspy),
	AT76_SET_HANDLER(SIOCGIWTHRSPY, at76_iw_handler_get_thrspy),
	AT76_SET_HANDLER(SIOCSIWAP, at76_iw_handler_set_wap),
	AT76_SET_HANDLER(SIOCGIWAP, at76_iw_handler_get_wap),
	AT76_SET_HANDLER(SIOCSIWSCAN, at76_iw_handler_set_scan),
	AT76_SET_HANDLER(SIOCGIWSCAN, at76_iw_handler_get_scan),
	AT76_SET_HANDLER(SIOCSIWESSID, at76_iw_handler_set_essid),
	AT76_SET_HANDLER(SIOCGIWESSID, at76_iw_handler_get_essid),
	AT76_SET_HANDLER(SIOCSIWRATE, at76_iw_handler_set_rate),
	AT76_SET_HANDLER(SIOCGIWRATE, at76_iw_handler_get_rate),
	AT76_SET_HANDLER(SIOCSIWRTS, at76_iw_handler_set_rts),
	AT76_SET_HANDLER(SIOCGIWRTS, at76_iw_handler_get_rts),
	AT76_SET_HANDLER(SIOCSIWFRAG, at76_iw_handler_set_frag),
	AT76_SET_HANDLER(SIOCGIWFRAG, at76_iw_handler_get_frag),
	AT76_SET_HANDLER(SIOCGIWTXPOW, at76_iw_handler_get_txpow),
	AT76_SET_HANDLER(SIOCSIWRETRY, at76_iw_handler_set_retry),
	AT76_SET_HANDLER(SIOCGIWRETRY, at76_iw_handler_get_retry),
	AT76_SET_HANDLER(SIOCSIWENCODE, at76_iw_handler_set_encode),
	AT76_SET_HANDLER(SIOCGIWENCODE, at76_iw_handler_get_encode),
	AT76_SET_HANDLER(SIOCSIWPOWER, at76_iw_handler_set_power),
	AT76_SET_HANDLER(SIOCGIWPOWER, at76_iw_handler_get_power)
};

#define AT76_SET_PRIV(h, f) [h - SIOCIWFIRSTPRIV] = (iw_handler) f

/* Private wireless handlers */
static const iw_handler at76_priv_handlers[] = {
	AT76_SET_PRIV(AT76_SET_SHORT_PREAMBLE, at76_iw_set_short_preamble),
	AT76_SET_PRIV(AT76_GET_SHORT_PREAMBLE, at76_iw_get_short_preamble),
	AT76_SET_PRIV(AT76_SET_DEBUG, at76_iw_set_debug),
	AT76_SET_PRIV(AT76_GET_DEBUG, at76_iw_get_debug),
	AT76_SET_PRIV(AT76_SET_POWERSAVE_MODE, at76_iw_set_powersave_mode),
	AT76_SET_PRIV(AT76_GET_POWERSAVE_MODE, at76_iw_get_powersave_mode),
	AT76_SET_PRIV(AT76_SET_SCAN_TIMES, at76_iw_set_scan_times),
	AT76_SET_PRIV(AT76_GET_SCAN_TIMES, at76_iw_get_scan_times),
	AT76_SET_PRIV(AT76_SET_SCAN_MODE, at76_iw_set_scan_mode),
	AT76_SET_PRIV(AT76_GET_SCAN_MODE, at76_iw_get_scan_mode),
	AT76_SET_PRIV(AT76_SET_INTL_ROAMING, at76_iw_set_intl_roaming),
	AT76_SET_PRIV(AT76_GET_INTL_ROAMING, at76_iw_get_intl_roaming),
};

/* Names and arguments of private wireless handlers */
static const struct iw_priv_args at76_priv_args[] = {
	/* 0 - long, 1 - short */
	{AT76_SET_SHORT_PREAMBLE,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_preamble"},

	{AT76_GET_SHORT_PREAMBLE,
	 0, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 10, "get_preamble"},

	/* we must pass the new debug mask as a string, because iwpriv cannot
	 * parse hex numbers starting with 0x :-(  */
	{AT76_SET_DEBUG,
	 IW_PRIV_TYPE_CHAR | 10, 0, "set_debug"},

	{AT76_GET_DEBUG,
	 0, IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | 10, "get_debug"},

	/* 1 - active, 2 - power save, 3 - smart power save */
	{AT76_SET_POWERSAVE_MODE,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_powersave"},

	{AT76_GET_POWERSAVE_MODE,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_powersave"},

	/* min_channel_time, max_channel_time */
	{AT76_SET_SCAN_TIMES,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "set_scan_times"},

	{AT76_GET_SCAN_TIMES,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, "get_scan_times"},

	/* 0 - active, 1 - passive scan */
	{AT76_SET_SCAN_MODE,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_scan_mode"},

	{AT76_GET_SCAN_MODE,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_scan_mode"},

	{AT76_SET_INTL_ROAMING,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_intl_scan"},

	{AT76_GET_INTL_ROAMING,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_intl_scan"}
};

static const struct iw_handler_def at76_handler_def = {
	.num_standard = ARRAY_SIZE(at76_handlers),
	.num_private = ARRAY_SIZE(at76_priv_handlers),
	.num_private_args = ARRAY_SIZE(at76_priv_args),
	.standard = at76_handlers,
	.private = at76_priv_handlers,
	.private_args = at76_priv_args,
	.get_wireless_stats = at76_get_wireless_stats,
};

static const u8 snapsig[] = { 0xaa, 0xaa, 0x03 };

/* RFC 1042 encapsulates Ethernet frames in 802.2 SNAP (0xaa, 0xaa, 0x03) with
 * a SNAP OID of 0 (0x00, 0x00, 0x00) */
static const u8 rfc1042sig[] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };

static int at76_tx(struct sk_buff *skb, struct net_device *netdev)
{
	struct at76_priv *priv = netdev_priv(netdev);
	struct net_device_stats *stats = &priv->stats;
	int ret = 0;
	int wlen;
	int submit_len;
	struct at76_tx_buffer *tx_buffer = priv->bulk_out_buffer;
	struct ieee80211_hdr_3addr *i802_11_hdr =
	    (struct ieee80211_hdr_3addr *)tx_buffer->packet;
	u8 *payload = i802_11_hdr->payload;
	struct ethhdr *eh = (struct ethhdr *)skb->data;

	if (netif_queue_stopped(netdev)) {
		err("%s: %s called while netdev is stopped", netdev->name,
		    __func__);
		/* skip this packet */
		dev_kfree_skb(skb);
		return 0;
	}

	if (priv->write_urb->status == -EINPROGRESS) {
		err("%s: %s called while priv->write_urb is pending for tx",
		    netdev->name, __func__);
		/* skip this packet */
		dev_kfree_skb(skb);
		return 0;
	}

	if (skb->len < ETH_HLEN) {
		err("%s: %s: skb too short (%d)", priv->netdev->name,
		    __func__, skb->len);
		dev_kfree_skb(skb);
		return 0;
	}

	at76_ledtrig_tx_activity();	/* tell ledtrigger we send a packet */

	/* we can get rid of memcpy if we set netdev->hard_header_len to
	   reserve enough space, but we would need to keep the skb around */

	if (ntohs(eh->h_proto) <= ETH_DATA_LEN) {
		/* this is a 802.3 packet */
		if (skb->len >= ETH_HLEN + sizeof(rfc1042sig)
		    && skb->data[ETH_HLEN] == rfc1042sig[0]
		    && skb->data[ETH_HLEN + 1] == rfc1042sig[1]) {
			/* higher layer delivered SNAP header - keep it */
			memcpy(payload, skb->data + ETH_HLEN,
			       skb->len - ETH_HLEN);
			wlen = IEEE80211_3ADDR_LEN + skb->len - ETH_HLEN;
		} else {
			err("%s: %s: no support for non-SNAP 802.2 packets "
			    "(DSAP 0x%02x SSAP 0x%02x cntrl 0x%02x)",
			    priv->netdev->name, __func__,
			    skb->data[ETH_HLEN], skb->data[ETH_HLEN + 1],
			    skb->data[ETH_HLEN + 2]);
			dev_kfree_skb(skb);
			return 0;
		}
	} else {
		/* add RFC 1042 header in front */
		memcpy(payload, rfc1042sig, sizeof(rfc1042sig));
		memcpy(payload + sizeof(rfc1042sig), &eh->h_proto,
		       skb->len - offsetof(struct ethhdr, h_proto));
		wlen = IEEE80211_3ADDR_LEN + sizeof(rfc1042sig) + skb->len -
		    offsetof(struct ethhdr, h_proto);
	}

	/* make wireless header */
	i802_11_hdr->frame_ctl =
	    cpu_to_le16(IEEE80211_FTYPE_DATA |
			(priv->wep_enabled ? IEEE80211_FCTL_PROTECTED : 0) |
			(priv->iw_mode ==
			 IW_MODE_INFRA ? IEEE80211_FCTL_TODS : 0));

	if (priv->iw_mode == IW_MODE_ADHOC) {
		memcpy(i802_11_hdr->addr1, eh->h_dest, ETH_ALEN);
		memcpy(i802_11_hdr->addr2, eh->h_source, ETH_ALEN);
		memcpy(i802_11_hdr->addr3, priv->bssid, ETH_ALEN);
	} else if (priv->iw_mode == IW_MODE_INFRA) {
		memcpy(i802_11_hdr->addr1, priv->bssid, ETH_ALEN);
		memcpy(i802_11_hdr->addr2, eh->h_source, ETH_ALEN);
		memcpy(i802_11_hdr->addr3, eh->h_dest, ETH_ALEN);
	}

	i802_11_hdr->duration_id = cpu_to_le16(0);
	i802_11_hdr->seq_ctl = cpu_to_le16(0);

	/* setup 'Atmel' header */
	tx_buffer->wlength = cpu_to_le16(wlen);
	tx_buffer->tx_rate = priv->txrate;
	/* for broadcast destination addresses, the firmware 0.100.x
	   seems to choose the highest rate set with CMD_STARTUP in
	   basic_rate_set replacing this value */

	memset(tx_buffer->reserved, 0, 4);

	tx_buffer->padding = at76_calc_padding(wlen);
	submit_len = wlen + AT76_TX_HDRLEN + tx_buffer->padding;

	at76_dbg(DBG_TX_DATA_CONTENT, "%s skb->data %s", priv->netdev->name,
		 hex2str(skb->data, 32));
	at76_dbg(DBG_TX_DATA, "%s tx: wlen 0x%x pad 0x%x rate %d hdr %s",
		 priv->netdev->name,
		 le16_to_cpu(tx_buffer->wlength),
		 tx_buffer->padding, tx_buffer->tx_rate,
		 hex2str(i802_11_hdr, sizeof(*i802_11_hdr)));
	at76_dbg(DBG_TX_DATA_CONTENT, "%s payload %s", priv->netdev->name,
		 hex2str(payload, 48));

	/* send stuff */
	netif_stop_queue(netdev);
	netdev->trans_start = jiffies;

	usb_fill_bulk_urb(priv->write_urb, priv->udev, priv->tx_bulk_pipe,
			  tx_buffer, submit_len, at76_write_bulk_callback,
			  priv);
	ret = usb_submit_urb(priv->write_urb, GFP_ATOMIC);
	if (ret) {
		stats->tx_errors++;
		err("%s: error in tx submit urb: %d", netdev->name, ret);
		if (ret == -EINVAL)
			err("-EINVAL: urb %p urb->hcpriv %p urb->complete %p",
			    priv->write_urb,
			    priv->write_urb ? priv->write_urb->
			    hcpriv : (void *)-1,
			    priv->write_urb ? priv->write_urb->
			    complete : (void *)-1);
	} else {
		stats->tx_bytes += skb->len;
		dev_kfree_skb(skb);
	}

	return ret;
}

static void at76_tx_timeout(struct net_device *netdev)
{
	struct at76_priv *priv = netdev_priv(netdev);

	if (!priv)
		return;
	warn("%s: tx timeout.", netdev->name);

	usb_unlink_urb(priv->write_urb);
	priv->stats.tx_errors++;
}

static int at76_submit_read_urb(struct at76_priv *priv)
{
	int ret;
	int size;
	struct sk_buff *skb = priv->rx_skb;

	if (priv->read_urb == NULL) {
		err("%s: priv->read_urb is NULL", __func__);
		return -EFAULT;
	}

	if (skb == NULL) {
		skb = dev_alloc_skb(sizeof(struct at76_rx_buffer));
		if (skb == NULL) {
			err("%s: unable to allocate rx skbuff.",
			    priv->netdev->name);
			ret = -ENOMEM;
			goto exit;
		}
		priv->rx_skb = skb;
	} else {
		skb_push(skb, skb_headroom(skb));
		skb_trim(skb, 0);
	}

	size = skb_tailroom(skb);
	usb_fill_bulk_urb(priv->read_urb, priv->udev, priv->rx_bulk_pipe,
			  skb_put(skb, size), size, at76_read_bulk_callback,
			  priv);
	ret = usb_submit_urb(priv->read_urb, GFP_ATOMIC);
	if (ret < 0) {
		if (ret == -ENODEV)
			at76_dbg(DBG_DEVSTART,
				 "usb_submit_urb returned -ENODEV");
		else
			err("%s: rx, usb_submit_urb failed: %d",
			    priv->netdev->name, ret);
	}

exit:
	if (ret < 0) {
		if (ret != -ENODEV) {
			/* If we can't submit the URB, the adapter becomes
			 * completely useless, so try again later */
			if (--priv->nr_submit_rx_tries > 0)
				schedule_work(&priv->work_submit_rx);
			else
				err("%s: giving up to submit rx urb after %d "
				    "failures - please unload the driver "
				    "and/or power cycle the device",
				    priv->netdev->name, NR_SUBMIT_RX_TRIES);
		}
	} else
		/* reset counter to initial value */
		priv->nr_submit_rx_tries = NR_SUBMIT_RX_TRIES;

	return ret;
}

static int at76_open(struct net_device *netdev)
{
	struct at76_priv *priv = netdev_priv(netdev);
	int ret = 0;

	at76_dbg(DBG_PROC_ENTRY, "%s(): entry", __func__);

	if (mutex_lock_interruptible(&priv->mtx))
		return -EINTR;

	/* if netdev->dev_addr != priv->mac_addr we must
	   set the mac address in the device ! */
	if (compare_ether_addr(netdev->dev_addr, priv->mac_addr)) {
		if (at76_add_mac_address(priv, netdev->dev_addr) >= 0)
			at76_dbg(DBG_PROGRESS, "%s: set new MAC addr %s",
				 netdev->name, mac2str(netdev->dev_addr));
	}
#ifdef DEBUG
	at76_dump_mib_mac_addr(priv);
#endif

	priv->scan_state = SCAN_IDLE;
	priv->last_scan = jiffies;
	priv->nr_submit_rx_tries = NR_SUBMIT_RX_TRIES;	/* init counter */

	ret = at76_submit_read_urb(priv);
	if (ret < 0) {
		err("%s: open: submit_read_urb failed: %d", netdev->name, ret);
		goto error;
	}

	tasklet_enable(&priv->rx_tasklet);
	schedule_delayed_work(&priv->dwork_restart, 0);

	at76_dbg(DBG_PROC_ENTRY, "%s(): end", __func__);
error:
	mutex_unlock(&priv->mtx);
	return ret < 0 ? ret : 0;
}

static int at76_stop(struct net_device *netdev)
{
	struct at76_priv *priv = netdev_priv(netdev);
	unsigned long flags;

	at76_dbg(DBG_DEVSTART, "%s: ENTER", __func__);

	if (mutex_lock_interruptible(&priv->mtx))
		return -EINTR;

	netif_stop_queue(netdev);

	at76_set_mac_state(priv, MAC_INIT);
	tasklet_disable(&priv->rx_tasklet);

	if (!priv->device_unplugged) {
		/* We are called by "ifconfig ethX down", not because the
		 * device is not available anymore. */
		at76_set_radio(priv, 0);

		/* We unlink read_urb because at76_open() re-submits it.
		 * If unplugged, at76_delete_device() takes care of it. */
		usb_kill_urb(priv->read_urb);
	}

	cancel_delayed_work(&priv->dwork_get_scan);
	cancel_delayed_work(&priv->dwork_beacon);
	cancel_delayed_work(&priv->dwork_auth);
	cancel_delayed_work(&priv->dwork_assoc);
	cancel_delayed_work(&priv->dwork_restart);

	spin_lock_irqsave(&priv->mgmt_spinlock, flags);
	kfree(priv->next_mgmt_bulk);
	priv->next_mgmt_bulk = NULL;
	spin_unlock_irqrestore(&priv->mgmt_spinlock, flags);

	/* free the bss_list */
	at76_free_bss_list(priv);

	mutex_unlock(&priv->mtx);
	at76_dbg(DBG_DEVSTART, "%s: EXIT", __func__);

	return 0;
}

static void at76_ethtool_get_drvinfo(struct net_device *netdev,
				     struct ethtool_drvinfo *info)
{
	struct at76_priv *priv = netdev_priv(netdev);

	strncpy(info->driver, DRIVER_NAME, sizeof(info->driver) - 1);

	strncpy(info->version, DRIVER_VERSION, sizeof(info->version));
	info->version[sizeof(info->version) - 1] = '\0';

	usb_make_path(priv->udev, info->bus_info, sizeof(info->bus_info));

	snprintf(info->fw_version, sizeof(info->fw_version) - 1, "%d.%d.%d-%d",
		 priv->fw_version.major, priv->fw_version.minor,
		 priv->fw_version.patch, priv->fw_version.build);
}

static u32 at76_ethtool_get_link(struct net_device *netdev)
{
	struct at76_priv *priv = netdev_priv(netdev);
	return priv->mac_state == MAC_CONNECTED;
}

static struct ethtool_ops at76_ethtool_ops = {
	.get_drvinfo = at76_ethtool_get_drvinfo,
	.get_link = at76_ethtool_get_link,
};

/* Register network device and initialize the hardware */
static int at76_init_new_device(struct at76_priv *priv,
				struct usb_interface *interface)
{
	struct net_device *netdev = priv->netdev;
	int ret;

	/* set up the endpoint information */
	/* check out the endpoints */

	at76_dbg(DBG_DEVSTART, "USB interface: %d endpoints",
		 interface->cur_altsetting->desc.bNumEndpoints);

	ret = at76_alloc_urbs(priv, interface);
	if (ret < 0)
		goto exit;

	/* MAC address */
	ret = at76_get_hw_config(priv);
	if (ret < 0) {
		err("could not get MAC address");
		goto exit;
	}

	priv->domain = at76_get_reg_domain(priv->regulatory_domain);
	/* init. netdev->dev_addr */
	memcpy(netdev->dev_addr, priv->mac_addr, ETH_ALEN);

	/* initializing */
	priv->international_roaming = IR_OFF;
	priv->channel = DEF_CHANNEL;
	priv->iw_mode = IW_MODE_INFRA;
	memset(priv->essid, 0, IW_ESSID_MAX_SIZE);
	priv->rts_threshold = DEF_RTS_THRESHOLD;
	priv->frag_threshold = DEF_FRAG_THRESHOLD;
	priv->short_retry_limit = DEF_SHORT_RETRY_LIMIT;
	priv->txrate = TX_RATE_AUTO;
	priv->preamble_type = PREAMBLE_TYPE_LONG;
	priv->beacon_period = 100;
	priv->beacons_last_qual = jiffies_to_msecs(jiffies);
	priv->auth_mode = WLAN_AUTH_OPEN;
	priv->scan_min_time = DEF_SCAN_MIN_TIME;
	priv->scan_max_time = DEF_SCAN_MAX_TIME;
	priv->scan_mode = SCAN_TYPE_ACTIVE;

	netdev->flags &= ~IFF_MULTICAST;	/* not yet or never */
	netdev->open = at76_open;
	netdev->stop = at76_stop;
	netdev->get_stats = at76_get_stats;
	netdev->ethtool_ops = &at76_ethtool_ops;

	/* Add pointers to enable iwspy support. */
	priv->wireless_data.spy_data = &priv->spy_data;
	netdev->wireless_data = &priv->wireless_data;

	netdev->hard_start_xmit = at76_tx;
	netdev->tx_timeout = at76_tx_timeout;
	netdev->watchdog_timeo = 2 * HZ;
	netdev->wireless_handlers = &at76_handler_def;
	netdev->set_multicast_list = at76_set_multicast;
	netdev->set_mac_address = at76_set_mac_address;

	ret = register_netdev(priv->netdev);
	if (ret) {
		err("unable to register netdevice %s (status %d)!",
		    priv->netdev->name, ret);
		goto exit;
	}
	priv->netdev_registered = 1;

	printk(KERN_INFO "%s: MAC address %s\n", netdev->name,
	       mac2str(priv->mac_addr));
	printk(KERN_INFO "%s: firmware version %d.%d.%d-%d\n", netdev->name,
	       priv->fw_version.major, priv->fw_version.minor,
	       priv->fw_version.patch, priv->fw_version.build);
	printk(KERN_INFO "%s: regulatory domain %s (id %d)\n", netdev->name,
	       priv->domain->name, priv->regulatory_domain);

	/* we let this timer run the whole time this driver instance lives */
	mod_timer(&priv->bss_list_timer, jiffies + BSS_LIST_TIMEOUT);

exit:
	return ret;
}

/* Download external firmware */
static int at76_load_external_fw(struct usb_device *udev, struct fwentry *fwe)
{
	int ret;
	int op_mode;

	op_mode = at76_get_op_mode(udev);
	at76_dbg(DBG_DEVSTART, "opmode %d", op_mode);

	if (op_mode != OPMODE_NORMAL_NIC_WITHOUT_FLASH) {
		err("unexpected opmode %d", op_mode);
		return -EINVAL;
	}

	if (!fwe->extfw || !fwe->extfw_size)
		return -ENOENT;

	ret = at76_download_external_fw(udev, fwe->extfw, fwe->extfw_size);
	if (ret < 0) {
		err("Downloading external firmware failed: %d", ret);
		return ret;
	}

	if (fwe->board_type == BOARD_505A_2958) {
		at76_dbg(DBG_DEVSTART, "200 ms delay for board type 7");
		schedule_timeout_interruptible(HZ / 5 + 1);
	}
	return 0;
}

/* Download internal firmware */
static int at76_load_internal_fw(struct usb_device *udev, struct fwentry *fwe)
{
	int ret;
	int need_remap = (fwe->board_type != BOARD_505A_2958);

	ret = at76_usbdfu_download(udev, fwe->intfw, fwe->intfw_size,
				   need_remap ? 0 : 2000);

	if (ret < 0) {
		err("downloading internal fw failed with %d", ret);
		goto exit;
	}

	at76_dbg(DBG_DEVSTART, "sending REMAP");

	/* no REMAP for 505A (see SF driver) */
	if (need_remap) {
		ret = at76_remap(udev);
		if (ret < 0) {
			err("sending REMAP failed with %d", ret);
			goto exit;
		}
	}

	at76_dbg(DBG_DEVSTART, "sleeping for 2 seconds");
	schedule_timeout_interruptible(2 * HZ + 1);
	usb_reset_device(udev);

exit:
	return ret;
}

static int at76_match_essid(struct at76_priv *priv, struct bss_info *ptr)
{
	/* common criteria for both modi */

	int ret = (priv->essid_size == 0 /* ANY ssid */  ||
		   (priv->essid_size == ptr->ssid_len &&
		    !memcmp(priv->essid, ptr->ssid, ptr->ssid_len)));
	if (!ret)
		at76_dbg(DBG_BSS_MATCH,
			 "%s bss table entry %p: essid didn't match",
			 priv->netdev->name, ptr);
	return ret;
}

static inline int at76_match_mode(struct at76_priv *priv, struct bss_info *ptr)
{
	int ret;

	if (priv->iw_mode == IW_MODE_ADHOC)
		ret = ptr->capa & WLAN_CAPABILITY_IBSS;
	else
		ret = ptr->capa & WLAN_CAPABILITY_ESS;
	if (!ret)
		at76_dbg(DBG_BSS_MATCH,
			 "%s bss table entry %p: mode didn't match",
			 priv->netdev->name, ptr);
	return ret;
}

static int at76_match_rates(struct at76_priv *priv, struct bss_info *ptr)
{
	int i;

	for (i = 0; i < ptr->rates_len; i++) {
		u8 rate = ptr->rates[i];

		if (!(rate & 0x80))
			continue;

		/* this is a basic rate we have to support
		   (see IEEE802.11, ch. 7.3.2.2) */
		if (rate != (0x80 | hw_rates[0])
		    && rate != (0x80 | hw_rates[1])
		    && rate != (0x80 | hw_rates[2])
		    && rate != (0x80 | hw_rates[3])) {
			at76_dbg(DBG_BSS_MATCH,
				 "%s: bss table entry %p: basic rate %02x not "
				 "supported", priv->netdev->name, ptr, rate);
			return 0;
		}
	}

	/* if we use short preamble, the bss must support it */
	if (priv->preamble_type == PREAMBLE_TYPE_SHORT &&
	    !(ptr->capa & WLAN_CAPABILITY_SHORT_PREAMBLE)) {
		at76_dbg(DBG_BSS_MATCH,
			 "%s: %p does not support short preamble",
			 priv->netdev->name, ptr);
		return 0;
	} else
		return 1;
}

static inline int at76_match_wep(struct at76_priv *priv, struct bss_info *ptr)
{
	if (!priv->wep_enabled && ptr->capa & WLAN_CAPABILITY_PRIVACY) {
		/* we have disabled WEP, but the BSS signals privacy */
		at76_dbg(DBG_BSS_MATCH,
			 "%s: bss table entry %p: requires encryption",
			 priv->netdev->name, ptr);
		return 0;
	}
	/* otherwise if the BSS does not signal privacy it may well
	   accept encrypted packets from us ... */
	return 1;
}

static inline int at76_match_bssid(struct at76_priv *priv, struct bss_info *ptr)
{
	if (!priv->wanted_bssid_valid ||
	    !compare_ether_addr(ptr->bssid, priv->wanted_bssid))
		return 1;

	at76_dbg(DBG_BSS_MATCH,
		 "%s: requested bssid - %s does not match",
		 priv->netdev->name, mac2str(priv->wanted_bssid));
	at76_dbg(DBG_BSS_MATCH,
		 "      AP bssid - %s of bss table entry %p",
		 mac2str(ptr->bssid), ptr);
	return 0;
}

/**
 * at76_match_bss - try to find a matching bss in priv->bss
 *
 * last - last bss tried
 *
 * last == NULL signals a new round starting with priv->bss_list.next
 * this function must be called inside an acquired priv->bss_list_spinlock
 * otherwise the timeout on bss may remove the newly chosen entry
 */
static struct bss_info *at76_match_bss(struct at76_priv *priv,
				       struct bss_info *last)
{
	struct bss_info *ptr = NULL;
	struct list_head *curr;

	curr = last != NULL ? last->list.next : priv->bss_list.next;
	while (curr != &priv->bss_list) {
		ptr = list_entry(curr, struct bss_info, list);
		if (at76_match_essid(priv, ptr) && at76_match_mode(priv, ptr)
		    && at76_match_wep(priv, ptr) && at76_match_rates(priv, ptr)
		    && at76_match_bssid(priv, ptr))
			break;
		curr = curr->next;
	}

	if (curr == &priv->bss_list)
		ptr = NULL;
	/* otherwise ptr points to the struct bss_info we have chosen */

	at76_dbg(DBG_BSS_TABLE, "%s %s: returned %p", priv->netdev->name,
		 __func__, ptr);
	return ptr;
}

/* Start joining a matching BSS, or create own IBSS */
static void at76_work_join(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      work_join);
	int ret;
	unsigned long flags;

	mutex_lock(&priv->mtx);

	WARN_ON(priv->mac_state != MAC_JOINING);
	if (priv->mac_state != MAC_JOINING)
		goto exit;

	/* secure the access to priv->curr_bss ! */
	spin_lock_irqsave(&priv->bss_list_spinlock, flags);
	priv->curr_bss = at76_match_bss(priv, priv->curr_bss);
	spin_unlock_irqrestore(&priv->bss_list_spinlock, flags);

	if (priv->curr_bss == NULL) {
		/* here we haven't found a matching (i)bss ... */
		if (priv->iw_mode == IW_MODE_ADHOC) {
			at76_set_mac_state(priv, MAC_OWN_IBSS);
			at76_start_ibss(priv);
			goto exit;
		}
		/* haven't found a matching BSS in infra mode - try again */
		at76_set_mac_state(priv, MAC_SCANNING);
		schedule_work(&priv->work_start_scan);
		goto exit;
	}

	ret = at76_join_bss(priv, priv->curr_bss);
	if (ret < 0) {
		err("%s: join_bss failed with %d", priv->netdev->name, ret);
		goto exit;
	}

	ret = at76_wait_completion(priv, CMD_JOIN);
	if (ret != CMD_STATUS_COMPLETE) {
		if (ret != CMD_STATUS_TIME_OUT)
			err("%s join_bss completed with %d",
			    priv->netdev->name, ret);
		else
			printk(KERN_INFO
			       "%s: join_bss ssid %s timed out\n",
			       priv->netdev->name,
			       mac2str(priv->curr_bss->bssid));

		/* retry next BSS immediately */
		schedule_work(&priv->work_join);
		goto exit;
	}

	/* here we have joined the (I)BSS */
	if (priv->iw_mode == IW_MODE_ADHOC) {
		struct bss_info *bptr = priv->curr_bss;
		at76_set_mac_state(priv, MAC_CONNECTED);
		/* get ESSID, BSSID and channel for priv->curr_bss */
		priv->essid_size = bptr->ssid_len;
		memcpy(priv->essid, bptr->ssid, bptr->ssid_len);
		memcpy(priv->bssid, bptr->bssid, ETH_ALEN);
		priv->channel = bptr->channel;
		at76_iwevent_bss_connect(priv->netdev, bptr->bssid);
		netif_carrier_on(priv->netdev);
		netif_start_queue(priv->netdev);
		/* just to be sure */
		cancel_delayed_work(&priv->dwork_get_scan);
		cancel_delayed_work(&priv->dwork_beacon);
		cancel_delayed_work(&priv->dwork_auth);
		cancel_delayed_work(&priv->dwork_assoc);
	} else {
		/* send auth req */
		priv->retries = AUTH_RETRIES;
		at76_set_mac_state(priv, MAC_AUTH);
		at76_auth_req(priv, priv->curr_bss, 1, NULL);
		at76_dbg(DBG_MGMT_TIMER,
			 "%s:%d: starting mgmt_timer + HZ", __func__, __LINE__);
		schedule_delayed_work(&priv->dwork_auth, AUTH_TIMEOUT);
	}

exit:
	mutex_unlock(&priv->mtx);
}

/* Reap scan results */
static void at76_dwork_get_scan(struct work_struct *work)
{
	int status;
	int ret;
	struct mib_mdomain mdomain;
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      dwork_get_scan.work);

	mutex_lock(&priv->mtx);
	WARN_ON(priv->mac_state != MAC_SCANNING);
	if (priv->mac_state != MAC_SCANNING)
		goto exit;

	status = at76_get_cmd_status(priv->udev, CMD_SCAN);
	if (status < 0) {
		err("%s: %s: at76_get_cmd_status failed with %d",
		    priv->netdev->name, __func__, status);
		status = CMD_STATUS_IN_PROGRESS;
		/* INFO: Hope it was a one off error - if not, scanning
		   further down the line and stop this cycle */
	}
	at76_dbg(DBG_PROGRESS,
		 "%s %s: got cmd_status %d (state %s, scan_runs %d)",
		 priv->netdev->name, __func__, status,
		 mac_states[priv->mac_state], priv->scan_runs);

	if (status != CMD_STATUS_COMPLETE) {
		if ((status != CMD_STATUS_IN_PROGRESS) &&
		    (status != CMD_STATUS_IDLE))
			err("%s: %s: Bad scan status: %s",
			    priv->netdev->name, __func__,
			    at76_get_cmd_status_string(status));

		/* the first cmd status after scan start is always a IDLE ->
		   start the timer to poll again until COMPLETED */
		at76_dbg(DBG_MGMT_TIMER,
			 "%s:%d: starting mgmt_timer for %d ticks",
			 __func__, __LINE__, SCAN_POLL_INTERVAL);
		schedule_delayed_work(&priv->dwork_get_scan,
				      SCAN_POLL_INTERVAL);
		goto exit;
	}

	if (at76_debug & DBG_BSS_TABLE)
		at76_dump_bss_table(priv);
	switch (priv->scan_runs) {

	case 1:
		WARN_ON(!priv->international_roaming);
		ret = at76_get_mib_mdomain(priv, &mdomain);
		if (ret < 0)
			err("at76_get_mib_mdomain returned %d", ret);
		else
			at76_dbg(DBG_MIB, "%s: MIB MDOMAIN: channel_list %s "
				 "tx_powerlevel %s", priv->netdev->name,
				 hex2str(mdomain.channel_list,
					 sizeof(mdomain.channel_list)),
				 hex2str(mdomain.tx_powerlevel,
					 sizeof(mdomain.tx_powerlevel)));
		ret = at76_start_scan(priv, 0, 1);
		if (ret < 0)
			err("%s: %s: start_scan (ANY) failed with %d",
			    priv->netdev->name, __func__, ret);
		at76_dbg(DBG_MGMT_TIMER,
			 "%s:%d: starting mgmt_timer for %d ticks",
			 __func__, __LINE__, SCAN_POLL_INTERVAL);
		schedule_delayed_work(&priv->dwork_get_scan,
				      SCAN_POLL_INTERVAL);
		break;

	case 2:
		ret = at76_start_scan(priv, 1, 1);
		if (ret < 0)
			err("%s: %s: start_scan (SSID) failed with %d",
			    priv->netdev->name, __func__, ret);
		at76_dbg(DBG_MGMT_TIMER,
			 "%s:%d: starting mgmt_timer for %d ticks",
			 __func__, __LINE__, SCAN_POLL_INTERVAL);
		schedule_delayed_work(&priv->dwork_get_scan,
				      SCAN_POLL_INTERVAL);
		break;

	case 3:
		priv->scan_state = SCAN_COMPLETED;
		/* report the end of scan to user space */
		at76_iwevent_scan_complete(priv->netdev);
		at76_set_mac_state(priv, MAC_JOINING);
		schedule_work(&priv->work_join);
		break;

	default:
		err("unexpected priv->scan_runs %d", priv->scan_runs);
	}

	priv->scan_runs++;

exit:
	mutex_unlock(&priv->mtx);
}

/* Handle loss of beacons from the AP */
static void at76_dwork_beacon(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      dwork_beacon.work);

	mutex_lock(&priv->mtx);
	WARN_ON(priv->mac_state != MAC_CONNECTED);
	if (priv->mac_state != MAC_CONNECTED)
		goto exit;

	/* We haven't received any beacons from out AP for BEACON_TIMEOUT */
	printk(KERN_INFO "%s: lost beacon bssid %s\n",
	       priv->netdev->name, mac2str(priv->curr_bss->bssid));

	/* jal: starting mgmt_timer in ad-hoc mode is questionable,
	   but I'll leave it here to debug another lockup problem */
	if (priv->iw_mode != IW_MODE_ADHOC) {
		netif_carrier_off(priv->netdev);
		netif_stop_queue(priv->netdev);
		at76_iwevent_bss_disconnect(priv->netdev);
		at76_set_mac_state(priv, MAC_SCANNING);
		schedule_work(&priv->work_start_scan);
	}

exit:
	mutex_unlock(&priv->mtx);
}

/* Handle authentication response timeout */
static void at76_dwork_auth(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      dwork_auth.work);

	mutex_lock(&priv->mtx);
	WARN_ON(priv->mac_state != MAC_AUTH);
	if (priv->mac_state != MAC_AUTH)
		goto exit;

	at76_dbg(DBG_PROGRESS, "%s: authentication response timeout",
		 priv->netdev->name);

	if (priv->retries-- >= 0) {
		at76_auth_req(priv, priv->curr_bss, 1, NULL);
		at76_dbg(DBG_MGMT_TIMER, "%s:%d: starting mgmt_timer + HZ",
			 __func__, __LINE__);
		schedule_delayed_work(&priv->dwork_auth, AUTH_TIMEOUT);
	} else {
		/* try to get next matching BSS */
		at76_set_mac_state(priv, MAC_JOINING);
		schedule_work(&priv->work_join);
	}

exit:
	mutex_unlock(&priv->mtx);
}

/* Handle association response timeout */
static void at76_dwork_assoc(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      dwork_assoc.work);

	mutex_lock(&priv->mtx);
	WARN_ON(priv->mac_state != MAC_ASSOC);
	if (priv->mac_state != MAC_ASSOC)
		goto exit;

	at76_dbg(DBG_PROGRESS, "%s: association response timeout",
		 priv->netdev->name);

	if (priv->retries-- >= 0) {
		at76_assoc_req(priv, priv->curr_bss);
		at76_dbg(DBG_MGMT_TIMER, "%s:%d: starting mgmt_timer + HZ",
			 __func__, __LINE__);
		schedule_delayed_work(&priv->dwork_assoc, ASSOC_TIMEOUT);
	} else {
		/* try to get next matching BSS */
		at76_set_mac_state(priv, MAC_JOINING);
		schedule_work(&priv->work_join);
	}

exit:
	mutex_unlock(&priv->mtx);
}

/* Read new bssid in ad-hoc mode */
static void at76_work_new_bss(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      work_new_bss);
	int ret;
	struct net_device *netdev = priv->netdev;
	struct mib_mac_mgmt mac_mgmt;

	mutex_lock(&priv->mtx);

	ret = at76_get_mib(priv->udev, MIB_MAC_MGMT, &mac_mgmt,
			   sizeof(struct mib_mac_mgmt));
	if (ret < 0) {
		err("%s: at76_get_mib failed: %d", netdev->name, ret);
		goto exit;
	}

	at76_dbg(DBG_PROGRESS, "ibss_change = 0x%2x", mac_mgmt.ibss_change);
	memcpy(priv->bssid, mac_mgmt.current_bssid, ETH_ALEN);
	at76_dbg(DBG_PROGRESS, "using BSSID %s", mac2str(priv->bssid));

	at76_iwevent_bss_connect(priv->netdev, priv->bssid);

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_MAC_MGMT;
	priv->mib_buf.size = 1;
	priv->mib_buf.index = offsetof(struct mib_mac_mgmt, ibss_change);
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (ibss change ok) failed: %d", netdev->name,
		    ret);

exit:
	mutex_unlock(&priv->mtx);
}

static int at76_startup_device(struct at76_priv *priv)
{
	struct at76_card_config *ccfg = &priv->card_config;
	int ret;

	if (at76_debug & DBG_PARAMS) {
		char ossid[IW_ESSID_MAX_SIZE + 1];

		/* make priv->essid printable */
		BUG_ON(priv->essid_size > IW_ESSID_MAX_SIZE);
		memcpy(ossid, priv->essid, priv->essid_size);
		ossid[priv->essid_size] = '\0';

		dbg("%s param: ssid %s (%s) mode %s ch %d wep %s key %d "
		    "keylen %d", priv->netdev->name, ossid,
		    hex2str(priv->essid, IW_ESSID_MAX_SIZE),
		    priv->iw_mode == IW_MODE_ADHOC ? "adhoc" : "infra",
		    priv->channel, priv->wep_enabled ? "enabled" : "disabled",
		    priv->wep_key_id, priv->wep_keys_len[priv->wep_key_id]);
		dbg("%s param: preamble %s rts %d retry %d frag %d "
		    "txrate %s auth_mode %d", priv->netdev->name,
		    preambles[priv->preamble_type], priv->rts_threshold,
		    priv->short_retry_limit, priv->frag_threshold,
		    priv->txrate == TX_RATE_1MBIT ? "1MBit" : priv->txrate ==
		    TX_RATE_2MBIT ? "2MBit" : priv->txrate ==
		    TX_RATE_5_5MBIT ? "5.5MBit" : priv->txrate ==
		    TX_RATE_11MBIT ? "11MBit" : priv->txrate ==
		    TX_RATE_AUTO ? "auto" : "<invalid>", priv->auth_mode);
		dbg("%s param: pm_mode %d pm_period %d auth_mode %s "
		    "scan_times %d %d scan_mode %s international_roaming %d",
		    priv->netdev->name, priv->pm_mode, priv->pm_period,
		    priv->auth_mode ==
		    WLAN_AUTH_OPEN ? "open" : "shared_secret",
		    priv->scan_min_time, priv->scan_max_time,
		    priv->scan_mode == SCAN_TYPE_ACTIVE ? "active" : "passive",
		    priv->international_roaming);
	}

	memset(ccfg, 0, sizeof(struct at76_card_config));
	ccfg->promiscuous_mode = 0;
	ccfg->short_retry_limit = priv->short_retry_limit;

	if (priv->wep_enabled) {
		if (priv->wep_keys_len[priv->wep_key_id] > WEP_SMALL_KEY_LEN)
			ccfg->encryption_type = 2;
		else
			ccfg->encryption_type = 1;

		/* jal: always exclude unencrypted if WEP is active */
		ccfg->exclude_unencrypted = 1;
	} else {
		ccfg->exclude_unencrypted = 0;
		ccfg->encryption_type = 0;
	}

	ccfg->rts_threshold = cpu_to_le16(priv->rts_threshold);
	ccfg->fragmentation_threshold = cpu_to_le16(priv->frag_threshold);

	memcpy(ccfg->basic_rate_set, hw_rates, 4);
	/* jal: really needed, we do a set_mib for autorate later ??? */
	ccfg->auto_rate_fallback = (priv->txrate == TX_RATE_AUTO ? 1 : 0);
	ccfg->channel = priv->channel;
	ccfg->privacy_invoked = priv->wep_enabled;
	memcpy(ccfg->current_ssid, priv->essid, IW_ESSID_MAX_SIZE);
	ccfg->ssid_len = priv->essid_size;

	ccfg->wep_default_key_id = priv->wep_key_id;
	memcpy(ccfg->wep_default_key_value, priv->wep_keys, 4 * WEP_KEY_LEN);

	ccfg->short_preamble = priv->preamble_type;
	ccfg->beacon_period = cpu_to_le16(priv->beacon_period);

	ret = at76_set_card_command(priv->udev, CMD_STARTUP, &priv->card_config,
				    sizeof(struct at76_card_config));
	if (ret < 0) {
		err("%s: at76_set_card_command failed: %d", priv->netdev->name,
		    ret);
		return ret;
	}

	at76_wait_completion(priv, CMD_STARTUP);

	/* remove BSSID from previous run */
	memset(priv->bssid, 0, ETH_ALEN);

	if (at76_set_radio(priv, 1) == 1)
		at76_wait_completion(priv, CMD_RADIO);

	ret = at76_set_preamble(priv, priv->preamble_type);
	if (ret < 0)
		return ret;

	ret = at76_set_frag(priv, priv->frag_threshold);
	if (ret < 0)
		return ret;

	ret = at76_set_rts(priv, priv->rts_threshold);
	if (ret < 0)
		return ret;

	ret = at76_set_autorate_fallback(priv,
					 priv->txrate == TX_RATE_AUTO ? 1 : 0);
	if (ret < 0)
		return ret;

	ret = at76_set_pm_mode(priv);
	if (ret < 0)
		return ret;

	ret = at76_set_iroaming(priv, priv->international_roaming);
	if (ret < 0)
		return ret;

	at76_set_monitor_mode(priv);

	if (at76_debug & DBG_MIB) {
		at76_dump_mib_mac(priv);
		at76_dump_mib_mac_addr(priv);
		at76_dump_mib_mac_mgmt(priv);
		at76_dump_mib_mac_wep(priv);
		at76_dump_mib_mdomain(priv);
		at76_dump_mib_phy(priv);
		at76_dump_mib_local(priv);
	}

	return 0;
}

/* Restart the interface */
static void at76_dwork_restart(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      dwork_restart.work);

	mutex_lock(&priv->mtx);

	at76_startup_device(priv);
	netif_carrier_off(priv->netdev);	/* stop netdev watchdog */
	netif_stop_queue(priv->netdev);	/* stop tx data packets */

	if (priv->iw_mode != IW_MODE_MONITOR) {
		at76_set_mac_state(priv, MAC_SCANNING);
		schedule_work(&priv->work_start_scan);
	} else
		at76_start_monitor(priv);

	mutex_unlock(&priv->mtx);
}

/* Initiate scanning */
static void at76_work_start_scan(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      work_start_scan);
	int ret;

	mutex_lock(&priv->mtx);

	WARN_ON(priv->mac_state != MAC_SCANNING);
	if (priv->mac_state != MAC_SCANNING)
		goto exit;

	/* only clear the bss list when a scan is actively initiated,
	 * otherwise simply rely on at76_bss_list_timeout */
	if (priv->scan_state == SCAN_IN_PROGRESS)
		at76_free_bss_list(priv);

	priv->scan_runs = 2;
	ret = at76_start_scan(priv, 0, 1);
	if (ret < 0)
		err("%s: %s: start_scan failed with %d",
		    priv->netdev->name, __func__, ret);
	else {
		at76_dbg(DBG_MGMT_TIMER,
			 "%s:%d: starting mgmt_timer for %d ticks",
			 __func__, __LINE__, SCAN_POLL_INTERVAL);
		schedule_delayed_work(&priv->dwork_get_scan,
				      SCAN_POLL_INTERVAL);
	}

exit:
	mutex_unlock(&priv->mtx);
}

/* Enable or disable promiscuous mode */
static void at76_work_set_promisc(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      work_set_promisc);
	int ret = 0;

	mutex_lock(&priv->mtx);

	memset(&priv->mib_buf, 0, sizeof(struct set_mib_buffer));
	priv->mib_buf.type = MIB_LOCAL;
	priv->mib_buf.size = 1;
	priv->mib_buf.index = offsetof(struct mib_local, promiscuous_mode);
	priv->mib_buf.data[0] = priv->promisc ? 1 : 0;
	ret = at76_set_mib(priv, &priv->mib_buf);
	if (ret < 0)
		err("%s: set_mib (promiscuous_mode) failed: %d",
		    priv->netdev->name, ret);

	mutex_unlock(&priv->mtx);
}

/* Submit Rx urb back to the device */
static void at76_work_submit_rx(struct work_struct *work)
{
	struct at76_priv *priv = container_of(work, struct at76_priv,
					      work_submit_rx);

	mutex_lock(&priv->mtx);
	at76_submit_read_urb(priv);
	mutex_unlock(&priv->mtx);
}

/* We got an association response */
static void at76_rx_mgmt_assoc(struct at76_priv *priv,
			       struct at76_rx_buffer *buf)
{
	struct ieee80211_assoc_response *resp =
	    (struct ieee80211_assoc_response *)buf->packet;
	struct ieee80211_hdr_3addr *mgmt = &resp->header;
	u16 assoc_id = le16_to_cpu(resp->aid);
	u16 status = le16_to_cpu(resp->status);
	u16 capa = le16_to_cpu(resp->capability);
	at76_dbg(DBG_RX_MGMT,
		 "%s: rx AssocResp bssid %s capa 0x%04x status 0x%04x "
		 "assoc_id 0x%04x rates %s", priv->netdev->name,
		 mac2str(mgmt->addr3), capa, status, assoc_id,
		 hex2str(resp->info_element->data, resp->info_element->len));

	if (priv->mac_state != MAC_ASSOC) {
		printk(KERN_INFO "%s: AssocResp in state %s ignored\n",
		       priv->netdev->name, mac_states[priv->mac_state]);
		return;
	}

	BUG_ON(priv->curr_bss == NULL);

	if (status == WLAN_STATUS_SUCCESS) {
		struct bss_info *ptr = priv->curr_bss;
		priv->assoc_id = assoc_id & 0x3fff;
		/* update iwconfig params */
		memcpy(priv->bssid, ptr->bssid, ETH_ALEN);
		memcpy(priv->essid, ptr->ssid, ptr->ssid_len);
		priv->essid_size = ptr->ssid_len;
		priv->channel = ptr->channel;
		schedule_work(&priv->work_assoc_done);
	} else {
		at76_set_mac_state(priv, MAC_JOINING);
		schedule_work(&priv->work_join);
	}
	cancel_delayed_work(&priv->dwork_get_scan);
	cancel_delayed_work(&priv->dwork_beacon);
	cancel_delayed_work(&priv->dwork_auth);
	cancel_delayed_work(&priv->dwork_assoc);
}

/* Process disassociation request from the AP */
static void at76_rx_mgmt_disassoc(struct at76_priv *priv,
				  struct at76_rx_buffer *buf)
{
	struct ieee80211_disassoc *resp =
	    (struct ieee80211_disassoc *)buf->packet;
	struct ieee80211_hdr_3addr *mgmt = &resp->header;

	at76_dbg(DBG_RX_MGMT,
		 "%s: rx DisAssoc bssid %s reason 0x%04x destination %s",
		 priv->netdev->name, mac2str(mgmt->addr3),
		 le16_to_cpu(resp->reason), mac2str(mgmt->addr1));

	/* We are not connected, ignore */
	if (priv->mac_state == MAC_SCANNING || priv->mac_state == MAC_INIT
	    || priv->curr_bss == NULL)
		return;

	/* Not our BSSID, ignore */
	if (compare_ether_addr(mgmt->addr3, priv->curr_bss->bssid))
		return;

	/* Not for our STA and not broadcast, ignore */
	if (compare_ether_addr(priv->netdev->dev_addr, mgmt->addr1)
	    && !is_broadcast_ether_addr(mgmt->addr1))
		return;

	if (priv->mac_state != MAC_ASSOC && priv->mac_state != MAC_CONNECTED
	    && priv->mac_state != MAC_JOINING) {
		printk(KERN_INFO "%s: DisAssoc in state %s ignored\n",
		       priv->netdev->name, mac_states[priv->mac_state]);
		return;
	}

	if (priv->mac_state == MAC_CONNECTED) {
		netif_carrier_off(priv->netdev);
		netif_stop_queue(priv->netdev);
		at76_iwevent_bss_disconnect(priv->netdev);
	}
	cancel_delayed_work(&priv->dwork_get_scan);
	cancel_delayed_work(&priv->dwork_beacon);
	cancel_delayed_work(&priv->dwork_auth);
	cancel_delayed_work(&priv->dwork_assoc);
	at76_set_mac_state(priv, MAC_JOINING);
	schedule_work(&priv->work_join);
}

static void at76_rx_mgmt_auth(struct at76_priv *priv,
			      struct at76_rx_buffer *buf)
{
	struct ieee80211_auth *resp = (struct ieee80211_auth *)buf->packet;
	struct ieee80211_hdr_3addr *mgmt = &resp->header;
	int seq_nr = le16_to_cpu(resp->transaction);
	int alg = le16_to_cpu(resp->algorithm);
	int status = le16_to_cpu(resp->status);

	at76_dbg(DBG_RX_MGMT,
		 "%s: rx AuthFrame bssid %s alg %d seq_nr %d status %d "
		 "destination %s", priv->netdev->name, mac2str(mgmt->addr3),
		 alg, seq_nr, status, mac2str(mgmt->addr1));

	if (alg == WLAN_AUTH_SHARED_KEY && seq_nr == 2)
		at76_dbg(DBG_RX_MGMT, "%s: AuthFrame challenge %s ...",
			 priv->netdev->name, hex2str(resp->info_element, 18));

	if (priv->mac_state != MAC_AUTH) {
		printk(KERN_INFO "%s: ignored AuthFrame in state %s\n",
		       priv->netdev->name, mac_states[priv->mac_state]);
		return;
	}
	if (priv->auth_mode != alg) {
		printk(KERN_INFO "%s: ignored AuthFrame for alg %d\n",
		       priv->netdev->name, alg);
		return;
	}

	BUG_ON(priv->curr_bss == NULL);

	/* Not our BSSID or not for our STA, ignore */
	if (compare_ether_addr(mgmt->addr3, priv->curr_bss->bssid)
	    || compare_ether_addr(priv->netdev->dev_addr, mgmt->addr1))
		return;

	if (status != WLAN_STATUS_SUCCESS) {
		cancel_delayed_work(&priv->dwork_get_scan);
		cancel_delayed_work(&priv->dwork_beacon);
		cancel_delayed_work(&priv->dwork_auth);
		cancel_delayed_work(&priv->dwork_assoc);
		/* try to join next bss */
		at76_set_mac_state(priv, MAC_JOINING);
		schedule_work(&priv->work_join);
		return;
	}

	if (priv->auth_mode == WLAN_AUTH_OPEN || seq_nr == 4) {
		priv->retries = ASSOC_RETRIES;
		at76_set_mac_state(priv, MAC_ASSOC);
		at76_assoc_req(priv, priv->curr_bss);
		at76_dbg(DBG_MGMT_TIMER,
			 "%s:%d: starting mgmt_timer + HZ", __func__, __LINE__);
		schedule_delayed_work(&priv->dwork_assoc, ASSOC_TIMEOUT);
		return;
	}

	WARN_ON(seq_nr != 2);
	at76_auth_req(priv, priv->curr_bss, seq_nr + 1, resp->info_element);
	at76_dbg(DBG_MGMT_TIMER, "%s:%d: starting mgmt_timer + HZ", __func__,
		 __LINE__);
	schedule_delayed_work(&priv->dwork_auth, AUTH_TIMEOUT);
}

static void at76_rx_mgmt_deauth(struct at76_priv *priv,
				struct at76_rx_buffer *buf)
{
	struct ieee80211_disassoc *resp =
	    (struct ieee80211_disassoc *)buf->packet;
	struct ieee80211_hdr_3addr *mgmt = &resp->header;

	at76_dbg(DBG_RX_MGMT | DBG_PROGRESS,
		 "%s: rx DeAuth bssid %s reason 0x%04x destination %s",
		 priv->netdev->name, mac2str(mgmt->addr3),
		 le16_to_cpu(resp->reason), mac2str(mgmt->addr1));

	if (priv->mac_state != MAC_AUTH && priv->mac_state != MAC_ASSOC
	    && priv->mac_state != MAC_CONNECTED) {
		printk(KERN_INFO "%s: DeAuth in state %s ignored\n",
		       priv->netdev->name, mac_states[priv->mac_state]);
		return;
	}

	BUG_ON(priv->curr_bss == NULL);

	/* Not our BSSID, ignore */
	if (compare_ether_addr(mgmt->addr3, priv->curr_bss->bssid))
		return;

	/* Not for our STA and not broadcast, ignore */
	if (compare_ether_addr(priv->netdev->dev_addr, mgmt->addr1)
	    && !is_broadcast_ether_addr(mgmt->addr1))
		return;

	if (priv->mac_state == MAC_CONNECTED)
		at76_iwevent_bss_disconnect(priv->netdev);

	at76_set_mac_state(priv, MAC_JOINING);
	schedule_work(&priv->work_join);
	cancel_delayed_work(&priv->dwork_get_scan);
	cancel_delayed_work(&priv->dwork_beacon);
	cancel_delayed_work(&priv->dwork_auth);
	cancel_delayed_work(&priv->dwork_assoc);
}

static void at76_rx_mgmt_beacon(struct at76_priv *priv,
				struct at76_rx_buffer *buf)
{
	int varpar_len;
	/* beacon content */
	struct ieee80211_beacon *bdata = (struct ieee80211_beacon *)buf->packet;
	struct ieee80211_hdr_3addr *mgmt = &bdata->header;

	struct list_head *lptr;
	struct bss_info *match;	/* entry matching addr3 with its bssid */
	int new_entry = 0;
	int len;
	struct ieee80211_info_element *tlv;
	int have_ssid = 0;
	int have_rates = 0;
	int have_channel = 0;
	int keep_going = 1;
	unsigned long flags;

	spin_lock_irqsave(&priv->bss_list_spinlock, flags);
	if (priv->mac_state == MAC_CONNECTED) {
		/* in state MAC_CONNECTED we use the mgmt_timer to control
		   the beacon of the BSS */
		BUG_ON(priv->curr_bss == NULL);

		if (!compare_ether_addr(priv->curr_bss->bssid, mgmt->addr3)) {
			/* We got our AP's beacon, defer the timeout handler.
			   Kill pending work first, as schedule_delayed_work()
			   won't do it. */
			cancel_delayed_work(&priv->dwork_beacon);
			schedule_delayed_work(&priv->dwork_beacon,
					      BEACON_TIMEOUT);
			priv->curr_bss->rssi = buf->rssi;
			priv->beacons_received++;
			goto exit;
		}
	}

	/* look if we have this BSS already in the list */
	match = NULL;

	if (!list_empty(&priv->bss_list)) {
		list_for_each(lptr, &priv->bss_list) {
			struct bss_info *bss_ptr =
			    list_entry(lptr, struct bss_info, list);
			if (!compare_ether_addr(bss_ptr->bssid, mgmt->addr3)) {
				match = bss_ptr;
				break;
			}
		}
	}

	if (match == NULL) {
		/* BSS not in the list - append it */
		match = kmalloc(sizeof(struct bss_info), GFP_ATOMIC);
		if (!match) {
			at76_dbg(DBG_BSS_TABLE,
				 "%s: cannot kmalloc new bss info (%zd byte)",
				 priv->netdev->name, sizeof(struct bss_info));
			goto exit;
		}
		memset(match, 0, sizeof(*match));
		new_entry = 1;
		list_add_tail(&match->list, &priv->bss_list);
	}

	match->capa = le16_to_cpu(bdata->capability);
	match->beacon_interval = le16_to_cpu(bdata->beacon_interval);
	match->rssi = buf->rssi;
	match->link_qual = buf->link_quality;
	match->noise_level = buf->noise_level;
	memcpy(match->bssid, mgmt->addr3, ETH_ALEN);
	at76_dbg(DBG_RX_BEACON, "%s: bssid %s", priv->netdev->name,
		 mac2str(match->bssid));

	tlv = bdata->info_element;

	/* length of var length beacon parameters */
	varpar_len = min_t(int, le16_to_cpu(buf->wlength) -
			   sizeof(struct ieee80211_beacon),
			   BEACON_MAX_DATA_LENGTH);

	/* This routine steps through the bdata->data array to get
	 * some useful information about the access point.
	 * Currently, this implementation supports receipt of: SSID,
	 * supported transfer rates and channel, in any order, with some
	 * tolerance for intermittent unknown codes (although this
	 * functionality may not be necessary as the useful information will
	 * usually arrive in consecutively, but there have been some
	 * reports of some of the useful information fields arriving in a
	 * different order).
	 * It does not support any more IE types although MFIE_TYPE_TIM may
	 * be supported (on my AP at least).
	 * The bdata->data array is about 1500 bytes long but only ~36 of those
	 * bytes are useful, hence the have_ssid etc optimizations. */

	while (keep_going &&
	       ((&tlv->data[tlv->len] - (u8 *)bdata->info_element) <=
		varpar_len)) {

		switch (tlv->id) {

		case MFIE_TYPE_SSID:
			if (have_ssid)
				break;

			len = min_t(int, IW_ESSID_MAX_SIZE, tlv->len);

			/* we copy only if this is a new entry,
			   or the incoming SSID is not a hidden SSID. This
			   will protect us from overwriting a real SSID read
			   in a ProbeResponse with a hidden one from a
			   following beacon. */
			if (!new_entry && at76_is_hidden_ssid(tlv->data, len)) {
				have_ssid = 1;
				break;
			}

			match->ssid_len = len;
			memcpy(match->ssid, tlv->data, len);
			match->ssid[len] = '\0';	/* terminate the
							   string for
							   printing */
			at76_dbg(DBG_RX_BEACON, "%s: SSID - %s",
				 priv->netdev->name, match->ssid);
			have_ssid = 1;
			break;

		case MFIE_TYPE_RATES:
			if (have_rates)
				break;

			match->rates_len =
			    min_t(int, sizeof(match->rates), tlv->len);
			memcpy(match->rates, tlv->data, match->rates_len);
			have_rates = 1;
			at76_dbg(DBG_RX_BEACON, "%s: SUPPORTED RATES %s",
				 priv->netdev->name,
				 hex2str(tlv->data, tlv->len));
			break;

		case MFIE_TYPE_DS_SET:
			if (have_channel)
				break;

			match->channel = tlv->data[0];
			have_channel = 1;
			at76_dbg(DBG_RX_BEACON, "%s: CHANNEL - %d",
				 priv->netdev->name, match->channel);
			break;

		case MFIE_TYPE_CF_SET:
		case MFIE_TYPE_TIM:
		case MFIE_TYPE_IBSS_SET:
		default:
			at76_dbg(DBG_RX_BEACON, "%s: beacon IE id %d len %d %s",
				 priv->netdev->name, tlv->id, tlv->len,
				 hex2str(tlv->data, tlv->len));
			break;
		}

		/* advance to the next informational element */
		next_ie(&tlv);

		/* Optimization: after all, the bdata->data array is
		 * varpar_len bytes long, whereas we get all of the useful
		 * information after only ~36 bytes, this saves us a lot of
		 * time (and trouble as the remaining portion of the array
		 * could be full of junk)
		 * Comment this out if you want to see what other information
		 * comes from the AP - although little of it may be useful */
	}

	at76_dbg(DBG_RX_BEACON, "%s: Finished processing beacon data",
		 priv->netdev->name);

	match->last_rx = jiffies;	/* record last rx of beacon */

exit:
	spin_unlock_irqrestore(&priv->bss_list_spinlock, flags);
}

/* Calculate the link level from a given rx_buffer */
static void at76_calc_level(struct at76_priv *priv, struct at76_rx_buffer *buf,
			    struct iw_quality *qual)
{
	/* just a guess for now, might be different for other chips */
	int max_rssi = 42;

	qual->level = (buf->rssi * 100 / max_rssi);
	if (qual->level > 100)
		qual->level = 100;
	qual->updated |= IW_QUAL_LEVEL_UPDATED;
}

/* Calculate the link quality from a given rx_buffer */
static void at76_calc_qual(struct at76_priv *priv, struct at76_rx_buffer *buf,
			   struct iw_quality *qual)
{
	if ((priv->board_type == BOARD_503_ISL3861) ||
	    (priv->board_type == BOARD_503_ISL3863))
		qual->qual = buf->link_quality;
	else {
		unsigned long msec;

		/* Update qual at most once a second */
		msec = jiffies_to_msecs(jiffies) - priv->beacons_last_qual;
		if (msec < 1000)
			return;

		qual->qual = qual->level * priv->beacons_received *
		    priv->beacon_period / msec;

		priv->beacons_last_qual = jiffies_to_msecs(jiffies);
		priv->beacons_received = 0;
	}
	qual->qual = (qual->qual > 100) ? 100 : qual->qual;
	qual->updated |= IW_QUAL_QUAL_UPDATED;
}

/* Calculate the noise quality from a given rx_buffer */
static void at76_calc_noise(struct at76_priv *priv, struct at76_rx_buffer *buf,
			    struct iw_quality *qual)
{
	qual->noise = 0;
	qual->updated |= IW_QUAL_NOISE_INVALID;
}

static void at76_update_wstats(struct at76_priv *priv,
			       struct at76_rx_buffer *buf)
{
	struct iw_quality *qual = &priv->wstats.qual;

	if (buf->rssi && priv->mac_state == MAC_CONNECTED) {
		qual->updated = 0;
		at76_calc_level(priv, buf, qual);
		at76_calc_qual(priv, buf, qual);
		at76_calc_noise(priv, buf, qual);
	} else {
		qual->qual = 0;
		qual->level = 0;
		qual->noise = 0;
		qual->updated = IW_QUAL_ALL_INVALID;
	}
}

static void at76_rx_mgmt(struct at76_priv *priv, struct at76_rx_buffer *buf)
{
	struct ieee80211_hdr_3addr *mgmt =
	    (struct ieee80211_hdr_3addr *)buf->packet;
	u16 framectl = le16_to_cpu(mgmt->frame_ctl);

	/* update wstats */
	if (priv->mac_state != MAC_INIT && priv->mac_state != MAC_SCANNING) {
		/* jal: this is a dirty hack needed by Tim in ad-hoc mode */
		/* Data packets always seem to have a 0 link level, so we
		   only read link quality info from management packets.
		   Atmel driver actually averages the present, and previous
		   values, we just present the raw value at the moment - TJS */
		if (priv->iw_mode == IW_MODE_ADHOC ||
		    (priv->curr_bss != NULL
		     && !compare_ether_addr(mgmt->addr3,
					    priv->curr_bss->bssid)))
			at76_update_wstats(priv, buf);
	}

	at76_dbg(DBG_RX_MGMT_CONTENT, "%s rx mgmt framectl 0x%x %s",
		 priv->netdev->name, framectl,
		 hex2str(mgmt, le16_to_cpu(buf->wlength)));

	switch (framectl & IEEE80211_FCTL_STYPE) {
	case IEEE80211_STYPE_BEACON:
	case IEEE80211_STYPE_PROBE_RESP:
		at76_rx_mgmt_beacon(priv, buf);
		break;

	case IEEE80211_STYPE_ASSOC_RESP:
		at76_rx_mgmt_assoc(priv, buf);
		break;

	case IEEE80211_STYPE_DISASSOC:
		at76_rx_mgmt_disassoc(priv, buf);
		break;

	case IEEE80211_STYPE_AUTH:
		at76_rx_mgmt_auth(priv, buf);
		break;

	case IEEE80211_STYPE_DEAUTH:
		at76_rx_mgmt_deauth(priv, buf);
		break;

	default:
		printk(KERN_DEBUG "%s: ignoring frame with framectl 0x%04x\n",
		       priv->netdev->name, framectl);
	}

	return;
}

static void at76_dbg_dumpbuf(const char *tag, const u8 *buf, int size)
{
	int i;

	if (!at76_debug)
		return;

	for (i = 0; i < size; i++) {
		if ((i % 8) == 0) {
			if (i)
				pr_debug("\n");
			pr_debug(DRIVER_NAME ": %s: ", tag);
		}
		pr_debug("%02x ", buf[i]);
	}
	pr_debug("\n");
}

/* Convert the 802.11 header into an ethernet-style header, make skb
 * ready for consumption by netif_rx() */
static void at76_ieee80211_to_eth(struct sk_buff *skb, int iw_mode)
{
	struct ieee80211_hdr_3addr *i802_11_hdr;
	struct ethhdr *eth_hdr_p;
	u8 *src_addr;
	u8 *dest_addr;

	i802_11_hdr = (struct ieee80211_hdr_3addr *)skb->data;

	dbg("%s: ENTRY skb len %d data %s", __func__,
	    skb->len, hex2str(skb->data, 64));

	/* That would be the ethernet header if the hardware converted
	 * the frame for us.  Make sure the source and the destination
	 * match the 802.11 header.  Which hardware does it? */
	eth_hdr_p = (struct ethhdr *)skb_pull(skb, IEEE80211_3ADDR_LEN);

	dest_addr = i802_11_hdr->addr1;
	if (iw_mode == IW_MODE_ADHOC)
		src_addr = i802_11_hdr->addr2;
	else
		src_addr = i802_11_hdr->addr3;

	if (!compare_ether_addr(eth_hdr_p->h_source, src_addr) &&
	    !compare_ether_addr(eth_hdr_p->h_dest, dest_addr))
		/* Yes, we already have an ethernet header */
		skb_reset_mac_header(skb);
	else {
		u16 len;

		/* Need to build an ethernet header */
		if (!memcmp(skb->data, snapsig, sizeof(snapsig))) {
			/* SNAP frame - decapsulate, keep proto */
			skb_push(skb, offsetof(struct ethhdr, h_proto) -
				 sizeof(rfc1042sig));
			len = 0;
		} else {
			/* 802.3 frame, proto is length */
			len = skb->len;
			skb_push(skb, ETH_HLEN);
		}

		skb_reset_mac_header(skb);
		eth_hdr_p = eth_hdr(skb);
		/* This needs to be done in this order (eth_hdr_p->h_dest may
		 * overlap src_addr) */
		memcpy(eth_hdr_p->h_source, src_addr, ETH_ALEN);
		memcpy(eth_hdr_p->h_dest, dest_addr, ETH_ALEN);
		if (len)
			eth_hdr_p->h_proto = htons(len);
	}

	skb->protocol = eth_type_trans(skb, skb->dev);

	dbg("%s: EXIT skb da %s sa %s proto 0x%04x len %d data %s",
	    __func__, mac2str(eth_hdr(skb)->h_dest),
	    mac2str(eth_hdr(skb)->h_source), ntohs(skb->protocol), skb->len,
	    hex2str(skb->data, 64));
}

/* Check for fragmented data in priv->rx_skb. If the packet was no fragment
   or it was the last of a fragment set a skb containing the whole packet
   is returned for further processing. Otherwise we get NULL and are
   done and the packet is either stored inside the fragment buffer
   or thrown away.  Every returned skb starts with the ieee802_11 header
   and contains _no_ FCS at the end */
static struct sk_buff *at76_check_for_rx_frags(struct at76_priv *priv)
{
	struct sk_buff *skb = priv->rx_skb;
	struct at76_rx_buffer *buf = (struct at76_rx_buffer *)skb->data;
	struct ieee80211_hdr_3addr *i802_11_hdr =
	    (struct ieee80211_hdr_3addr *)buf->packet;
	/* seq_ctrl, fragment_number, sequence number of new packet */
	u16 sctl = le16_to_cpu(i802_11_hdr->seq_ctl);
	u16 fragnr = sctl & 0xf;
	u16 seqnr = sctl >> 4;
	u16 frame_ctl = le16_to_cpu(i802_11_hdr->frame_ctl);

	/* Length including the IEEE802.11 header, but without the trailing
	 * FCS and without the Atmel Rx header */
	int length = le16_to_cpu(buf->wlength) - IEEE80211_FCS_LEN;

	/* where does the data payload start in skb->data ? */
	u8 *data = i802_11_hdr->payload;

	/* length of payload, excl. the trailing FCS */
	int data_len = length - IEEE80211_3ADDR_LEN;

	int i;
	struct rx_data_buf *bptr, *optr;
	unsigned long oldest = ~0UL;

	at76_dbg(DBG_RX_FRAGS,
		 "%s: rx data frame_ctl %04x addr2 %s seq/frag %d/%d "
		 "length %d data %d: %s ...", priv->netdev->name, frame_ctl,
		 mac2str(i802_11_hdr->addr2), seqnr, fragnr, length, data_len,
		 hex2str(data, 32));

	at76_dbg(DBG_RX_FRAGS_SKB, "%s: incoming skb: head %p data %p "
		 "tail %p end %p len %d", priv->netdev->name, skb->head,
		 skb->data, skb_tail_pointer(skb), skb_end_pointer(skb),
		 skb->len);

	if (data_len < 0) {
		/* make sure data starts in the buffer */
		printk(KERN_INFO "%s: data frame too short\n",
		       priv->netdev->name);
		return NULL;
	}

	WARN_ON(length <= AT76_RX_HDRLEN);
	if (length <= AT76_RX_HDRLEN)
		return NULL;

	/* remove the at76_rx_buffer header - we don't need it anymore */
	/* we need the IEEE802.11 header (for the addresses) if this packet
	   is the first of a chain */
	skb_pull(skb, AT76_RX_HDRLEN);

	/* remove FCS at end */
	skb_trim(skb, length);

	at76_dbg(DBG_RX_FRAGS_SKB, "%s: trimmed skb: head %p data %p tail %p "
		 "end %p len %d data %p data_len %d", priv->netdev->name,
		 skb->head, skb->data, skb_tail_pointer(skb),
		 skb_end_pointer(skb), skb->len, data, data_len);

	if (fragnr == 0 && !(frame_ctl & IEEE80211_FCTL_MOREFRAGS)) {
		/* unfragmented packet received */
		/* Use a new skb for the next receive */
		priv->rx_skb = NULL;
		at76_dbg(DBG_RX_FRAGS, "%s: unfragmented", priv->netdev->name);
		return skb;
	}

	/* look if we've got a chain for the sender address.
	   afterwards optr points to first free or the oldest entry,
	   or, if i < NR_RX_DATA_BUF, bptr points to the entry for the
	   sender address */
	/* determining the oldest entry doesn't cope with jiffies wrapping
	   but I don't care to delete a young entry at these rare moments ... */

	bptr = priv->rx_data;
	optr = NULL;
	for (i = 0; i < NR_RX_DATA_BUF; i++, bptr++) {
		if (bptr->skb == NULL) {
			optr = bptr;
			oldest = 0UL;
			continue;
		}

		if (!compare_ether_addr(i802_11_hdr->addr2, bptr->sender))
			break;

		if (optr == NULL) {
			optr = bptr;
			oldest = bptr->last_rx;
		} else if (bptr->last_rx < oldest)
			optr = bptr;
	}

	if (i < NR_RX_DATA_BUF) {

		at76_dbg(DBG_RX_FRAGS, "%s: %d. cacheentry (seq/frag = %d/%d) "
			 "matched sender addr",
			 priv->netdev->name, i, bptr->seqnr, bptr->fragnr);

		/* bptr points to an entry for the sender address */
		if (bptr->seqnr == seqnr) {
			int left;
			/* the fragment has the current sequence number */
			if (((bptr->fragnr + 1) & 0xf) != fragnr) {
				/* wrong fragment number -> ignore it */
				/* is & 0xf necessary above ??? */
				at76_dbg(DBG_RX_FRAGS,
					 "%s: frag nr mismatch: %d + 1 != %d",
					 priv->netdev->name, bptr->fragnr,
					 fragnr);
				return NULL;
			}
			bptr->last_rx = jiffies;
			/* the next following fragment number ->
			   add the data at the end */

			/* for test only ??? */
			left = skb_tailroom(bptr->skb);
			if (left < data_len)
				printk(KERN_INFO
				       "%s: only %d byte free (need %d)\n",
				       priv->netdev->name, left, data_len);
			else
				memcpy(skb_put(bptr->skb, data_len), data,
				       data_len);

			bptr->fragnr = fragnr;
			if (frame_ctl & IEEE80211_FCTL_MOREFRAGS)
				return NULL;

			/* this was the last fragment - send it */
			skb = bptr->skb;
			bptr->skb = NULL;	/* free the entry */
			at76_dbg(DBG_RX_FRAGS, "%s: last frag of seq %d",
				 priv->netdev->name, seqnr);
			return skb;
		}

		/* got another sequence number */
		if (fragnr == 0) {
			/* it's the start of a new chain - replace the
			   old one by this */
			/* bptr->sender has the correct value already */
			at76_dbg(DBG_RX_FRAGS,
				 "%s: start of new seq %d, removing old seq %d",
				 priv->netdev->name, seqnr, bptr->seqnr);
			bptr->seqnr = seqnr;
			bptr->fragnr = 0;
			bptr->last_rx = jiffies;
			/* swap bptr->skb and priv->rx_skb */
			skb = bptr->skb;
			bptr->skb = priv->rx_skb;
			priv->rx_skb = skb;
		} else {
			/* it from the middle of a new chain ->
			   delete the old entry and skip the new one */
			at76_dbg(DBG_RX_FRAGS,
				 "%s: middle of new seq %d (%d) "
				 "removing old seq %d",
				 priv->netdev->name, seqnr, fragnr,
				 bptr->seqnr);
			dev_kfree_skb(bptr->skb);
			bptr->skb = NULL;
		}
		return NULL;
	}

	/* if we didn't find a chain for the sender address, optr
	   points either to the first free or the oldest entry */

	if (fragnr != 0) {
		/* this is not the begin of a fragment chain ... */
		at76_dbg(DBG_RX_FRAGS,
			 "%s: no chain for non-first fragment (%d)",
			 priv->netdev->name, fragnr);
		return NULL;
	}

	BUG_ON(optr == NULL);
	if (optr->skb != NULL) {
		/* swap the skb's */
		skb = optr->skb;
		optr->skb = priv->rx_skb;
		priv->rx_skb = skb;

		at76_dbg(DBG_RX_FRAGS,
			 "%s: free old contents: sender %s seq/frag %d/%d",
			 priv->netdev->name, mac2str(optr->sender),
			 optr->seqnr, optr->fragnr);

	} else {
		/* take the skb from priv->rx_skb */
		optr->skb = priv->rx_skb;
		/* let at76_submit_read_urb() allocate a new skb */
		priv->rx_skb = NULL;

		at76_dbg(DBG_RX_FRAGS, "%s: use a free entry",
			 priv->netdev->name);
	}
	memcpy(optr->sender, i802_11_hdr->addr2, ETH_ALEN);
	optr->seqnr = seqnr;
	optr->fragnr = 0;
	optr->last_rx = jiffies;

	return NULL;
}

/* Rx interrupt: we expect the complete data buffer in priv->rx_skb */
static void at76_rx_data(struct at76_priv *priv)
{
	struct net_device *netdev = priv->netdev;
	struct net_device_stats *stats = &priv->stats;
	struct sk_buff *skb = priv->rx_skb;
	struct at76_rx_buffer *buf = (struct at76_rx_buffer *)skb->data;
	struct ieee80211_hdr_3addr *i802_11_hdr;
	int length = le16_to_cpu(buf->wlength);

	at76_dbg(DBG_RX_DATA, "%s received data packet:", netdev->name);
	if (at76_debug & DBG_RX_DATA)
		at76_dbg_dumpbuf(" rxhdr", skb->data, AT76_RX_HDRLEN);

	if (at76_debug & DBG_RX_DATA_CONTENT)
		at76_dbg_dumpbuf("packet", skb->data + AT76_RX_HDRLEN, length);

	skb = at76_check_for_rx_frags(priv);
	if (skb == NULL)
		return;

	/* Atmel header and the FCS are already removed */
	i802_11_hdr = (struct ieee80211_hdr_3addr *)skb->data;

	skb->dev = netdev;
	skb->ip_summed = CHECKSUM_NONE;	/* TODO: should check CRC */

	if (is_broadcast_ether_addr(i802_11_hdr->addr1)) {
		if (!compare_ether_addr(i802_11_hdr->addr1, netdev->broadcast))
			skb->pkt_type = PACKET_BROADCAST;
		else
			skb->pkt_type = PACKET_MULTICAST;
	} else if (compare_ether_addr(i802_11_hdr->addr1, netdev->dev_addr))
		skb->pkt_type = PACKET_OTHERHOST;

	at76_ieee80211_to_eth(skb, priv->iw_mode);

	netdev->last_rx = jiffies;
	netif_rx(skb);
	stats->rx_packets++;
	stats->rx_bytes += length;

	return;
}

static void at76_rx_monitor_mode(struct at76_priv *priv)
{
	struct at76_rx_radiotap *rt;
	u8 *payload;
	int skblen;
	struct net_device *netdev = priv->netdev;
	struct at76_rx_buffer *buf =
	    (struct at76_rx_buffer *)priv->rx_skb->data;
	/* length including the IEEE802.11 header and the trailing FCS,
	   but not at76_rx_buffer */
	int length = le16_to_cpu(buf->wlength);
	struct sk_buff *skb = priv->rx_skb;
	struct net_device_stats *stats = &priv->stats;

	if (length < IEEE80211_FCS_LEN) {
		/* buffer contains no data */
		at76_dbg(DBG_MONITOR_MODE,
			 "%s: MONITOR MODE: rx skb without data",
			 priv->netdev->name);
		return;
	}

	skblen = sizeof(struct at76_rx_radiotap) + length;

	skb = dev_alloc_skb(skblen);
	if (skb == NULL) {
		err("%s: MONITOR MODE: dev_alloc_skb for radiotap header "
		    "returned NULL", priv->netdev->name);
		return;
	}

	skb_put(skb, skblen);

	rt = (struct at76_rx_radiotap *)skb->data;
	payload = skb->data + sizeof(struct at76_rx_radiotap);

	rt->rt_hdr.it_version = 0;
	rt->rt_hdr.it_pad = 0;
	rt->rt_hdr.it_len = cpu_to_le16(sizeof(struct at76_rx_radiotap));
	rt->rt_hdr.it_present = cpu_to_le32(AT76_RX_RADIOTAP_PRESENT);

	rt->rt_tsft = cpu_to_le64(le32_to_cpu(buf->rx_time));
	rt->rt_rate = hw_rates[buf->rx_rate] & (~0x80);
	rt->rt_signal = buf->rssi;
	rt->rt_noise = buf->noise_level;
	rt->rt_flags = IEEE80211_RADIOTAP_F_FCS;
	if (buf->fragmentation)
		rt->rt_flags |= IEEE80211_RADIOTAP_F_FRAG;

	memcpy(payload, buf->packet, length);
	skb->dev = netdev;
	skb->ip_summed = CHECKSUM_NONE;
	skb_reset_mac_header(skb);
	skb->pkt_type = PACKET_OTHERHOST;
	skb->protocol = htons(ETH_P_80211_RAW);

	netdev->last_rx = jiffies;
	netif_rx(skb);
	stats->rx_packets++;
	stats->rx_bytes += length;
}

/* Check if we spy on the sender address in buf and update stats */
static void at76_iwspy_update(struct at76_priv *priv,
			      struct at76_rx_buffer *buf)
{
	struct ieee80211_hdr_3addr *hdr =
	    (struct ieee80211_hdr_3addr *)buf->packet;
	struct iw_quality qual;

	/* We can only set the level here */
	qual.updated = IW_QUAL_QUAL_INVALID | IW_QUAL_NOISE_INVALID;
	qual.level = 0;
	qual.noise = 0;
	at76_calc_level(priv, buf, &qual);

	spin_lock_bh(&priv->spy_spinlock);

	if (priv->spy_data.spy_number > 0)
		wireless_spy_update(priv->netdev, hdr->addr2, &qual);

	spin_unlock_bh(&priv->spy_spinlock);
}

static void at76_rx_tasklet(unsigned long param)
{
	struct urb *urb = (struct urb *)param;
	struct at76_priv *priv = urb->context;
	struct net_device *netdev = priv->netdev;
	struct at76_rx_buffer *buf;
	struct ieee80211_hdr_3addr *i802_11_hdr;
	u16 frame_ctl;

	if (priv->device_unplugged) {
		at76_dbg(DBG_DEVSTART, "device unplugged");
		if (urb)
			at76_dbg(DBG_DEVSTART, "urb status %d", urb->status);
		return;
	}

	if (!priv->rx_skb || !netdev || !priv->rx_skb->data)
		return;

	buf = (struct at76_rx_buffer *)priv->rx_skb->data;

	i802_11_hdr = (struct ieee80211_hdr_3addr *)buf->packet;

	frame_ctl = le16_to_cpu(i802_11_hdr->frame_ctl);

	if (urb->status != 0) {
		if (urb->status != -ENOENT && urb->status != -ECONNRESET)
			at76_dbg(DBG_URB,
				 "%s %s: - nonzero Rx bulk status received: %d",
				 __func__, netdev->name, urb->status);
		return;
	}

	at76_dbg(DBG_RX_ATMEL_HDR,
		 "%s: rx frame: rate %d rssi %d noise %d link %d %s",
		 priv->netdev->name, buf->rx_rate, buf->rssi, buf->noise_level,
		 buf->link_quality, hex2str(i802_11_hdr, 48));
	if (priv->iw_mode == IW_MODE_MONITOR) {
		at76_rx_monitor_mode(priv);
		goto exit;
	}

	/* there is a new bssid around, accept it: */
	if (buf->newbss && priv->iw_mode == IW_MODE_ADHOC) {
		at76_dbg(DBG_PROGRESS, "%s: rx newbss", netdev->name);
		schedule_work(&priv->work_new_bss);
	}

	switch (frame_ctl & IEEE80211_FCTL_FTYPE) {
	case IEEE80211_FTYPE_DATA:
		at76_rx_data(priv);
		break;

	case IEEE80211_FTYPE_MGMT:
		/* jal: TODO: find out if we can update iwspy also on
		   other frames than management (might depend on the
		   radio chip / firmware version !) */

		at76_iwspy_update(priv, buf);

		at76_rx_mgmt(priv, buf);
		break;

	case IEEE80211_FTYPE_CTL:
		at76_dbg(DBG_RX_CTRL, "%s: ignored ctrl frame: %04x",
			 priv->netdev->name, frame_ctl);
		break;

	default:
		printk(KERN_DEBUG "%s: ignoring frame with framectl 0x%04x\n",
		       priv->netdev->name, frame_ctl);
	}
exit:
	at76_submit_read_urb(priv);
}

/* Allocate network device and initialize private data */
static struct at76_priv *at76_alloc_new_device(struct usb_device *udev)
{
	struct net_device *netdev;
	struct at76_priv *priv = NULL;
	int i;

	/* allocate memory for our device state and initialize it */
	netdev = alloc_etherdev(sizeof(struct at76_priv));
	if (netdev == NULL) {
		err("out of memory");
		return NULL;
	}

	priv = netdev_priv(netdev);
	memset(priv, 0, sizeof(*priv));

	priv->udev = udev;
	priv->netdev = netdev;

	mutex_init(&priv->mtx);
	INIT_WORK(&priv->work_assoc_done, at76_work_assoc_done);
	INIT_WORK(&priv->work_join, at76_work_join);
	INIT_WORK(&priv->work_new_bss, at76_work_new_bss);
	INIT_WORK(&priv->work_start_scan, at76_work_start_scan);
	INIT_WORK(&priv->work_set_promisc, at76_work_set_promisc);
	INIT_WORK(&priv->work_submit_rx, at76_work_submit_rx);
	INIT_DELAYED_WORK(&priv->dwork_restart, at76_dwork_restart);
	INIT_DELAYED_WORK(&priv->dwork_get_scan, at76_dwork_get_scan);
	INIT_DELAYED_WORK(&priv->dwork_beacon, at76_dwork_beacon);
	INIT_DELAYED_WORK(&priv->dwork_auth, at76_dwork_auth);
	INIT_DELAYED_WORK(&priv->dwork_assoc, at76_dwork_assoc);

	spin_lock_init(&priv->mgmt_spinlock);
	priv->next_mgmt_bulk = NULL;
	priv->mac_state = MAC_INIT;

	/* initialize empty BSS list */
	priv->curr_bss = NULL;
	INIT_LIST_HEAD(&priv->bss_list);
	spin_lock_init(&priv->bss_list_spinlock);

	init_timer(&priv->bss_list_timer);
	priv->bss_list_timer.data = (unsigned long)priv;
	priv->bss_list_timer.function = at76_bss_list_timeout;

	spin_lock_init(&priv->spy_spinlock);

	/* mark all rx data entries as unused */
	for (i = 0; i < NR_RX_DATA_BUF; i++)
		priv->rx_data[i].skb = NULL;

	priv->rx_tasklet.func = at76_rx_tasklet;
	priv->rx_tasklet.data = 0;
	tasklet_disable(&priv->rx_tasklet);

	priv->pm_mode = AT76_PM_OFF;
	priv->pm_period = 0;

	return priv;
}

/* Load firmware into kernel memory and parse it */
static struct fwentry *at76_load_firmware(struct usb_device *udev,
					  enum board_type board_type)
{
	int ret;
	char *str;
	struct at76_fw_header *fwh;
	struct fwentry *fwe = &firmwares[board_type];

	mutex_lock(&fw_mutex);

	if (fwe->loaded) {
		at76_dbg(DBG_FW, "re-using previously loaded fw");
		goto exit;
	}

	at76_dbg(DBG_FW, "downloading firmware %s", fwe->fwname);
	ret = request_firmware(&fwe->fw, fwe->fwname, &udev->dev);
	if (ret < 0) {
		err("firmware %s not found.", fwe->fwname);
		err("You may need to download the firmware from "
		    "http://developer.berlios.de/projects/at76c503a/");
		goto exit;
	}

	at76_dbg(DBG_FW, "got it.");
	fwh = (struct at76_fw_header *)(fwe->fw->data);

	if (fwe->fw->size <= sizeof(*fwh)) {
		err("firmware is too short (0x%zx)", fwe->fw->size);
		goto exit;
	}

	/* CRC currently not checked */
	fwe->board_type = le32_to_cpu(fwh->board_type);
	if (fwe->board_type != board_type) {
		err("board type mismatch, requested %u, got %u", board_type,
		    fwe->board_type);
		goto exit;
	}

	fwe->fw_version.major = fwh->major;
	fwe->fw_version.minor = fwh->minor;
	fwe->fw_version.patch = fwh->patch;
	fwe->fw_version.build = fwh->build;

	str = (char *)fwh + le32_to_cpu(fwh->str_offset);
	fwe->intfw = (u8 *)fwh + le32_to_cpu(fwh->int_fw_offset);
	fwe->intfw_size = le32_to_cpu(fwh->int_fw_len);
	fwe->extfw = (u8 *)fwh + le32_to_cpu(fwh->ext_fw_offset);
	fwe->extfw_size = le32_to_cpu(fwh->ext_fw_len);

	fwe->loaded = 1;

	at76_dbg(DBG_DEVSTART, "firmware board %u version %u.%u.%u-%u "
		 "(int %d:%d, ext %d:%d)", board_type,
		 fwh->major, fwh->minor, fwh->patch, fwh->build,
		 le32_to_cpu(fwh->int_fw_offset), le32_to_cpu(fwh->int_fw_len),
		 le32_to_cpu(fwh->ext_fw_offset), le32_to_cpu(fwh->ext_fw_len));
	at76_dbg(DBG_DEVSTART, "firmware id %s", str);

exit:
	mutex_unlock(&fw_mutex);

	if (fwe->loaded)
		return fwe;
	else
		return NULL;
}

static int at76_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	int ret;
	struct at76_priv *priv;
	struct fwentry *fwe;
	struct usb_device *udev;
	int op_mode;
	int need_ext_fw = 0;
	struct mib_fw_version fwv;
	int board_type = (int)id->driver_info;

	udev = usb_get_dev(interface_to_usbdev(interface));

	/* Load firmware into kernel memory */
	fwe = at76_load_firmware(udev, board_type);
	if (!fwe) {
		ret = -ENOENT;
		goto error;
	}

	op_mode = at76_get_op_mode(udev);

	at76_dbg(DBG_DEVSTART, "opmode %d", op_mode);

	/* we get OPMODE_NONE with 2.4.23, SMC2662W-AR ???
	   we get 204 with 2.4.23, Fiberline FL-WL240u (505A+RFMD2958) ??? */

	if (op_mode == OPMODE_HW_CONFIG_MODE) {
		err("cannot handle a device in HW_CONFIG_MODE (opmode %d)",
		    op_mode);
		ret = -EBUSY;
		goto error;
	}

	if (op_mode != OPMODE_NORMAL_NIC_WITH_FLASH
	    && op_mode != OPMODE_NORMAL_NIC_WITHOUT_FLASH) {
		/* download internal firmware part */
		at76_dbg(DBG_DEVSTART, "downloading internal firmware");
		ret = at76_load_internal_fw(udev, fwe);
		if (ret < 0) {
			err("error %d downloading internal firmware", ret);
			goto error;
		}
		usb_put_dev(udev);
		return ret;
	}

	/* Internal firmware already inside the device.  Get firmware
	 * version to test if external firmware is loaded.
	 * This works only for newer firmware, e.g. the Intersil 0.90.x
	 * says "control timeout on ep0in" and subsequent
	 * at76_get_op_mode() fail too :-( */

	/* if version >= 0.100.x.y or device with built-in flash we can
	 * query the device for the fw version */
	if ((fwe->fw_version.major > 0 || fwe->fw_version.minor >= 100)
	    || (op_mode == OPMODE_NORMAL_NIC_WITH_FLASH)) {
		ret = at76_get_mib(udev, MIB_FW_VERSION, &fwv, sizeof(fwv));
		if (ret < 0 || (fwv.major | fwv.minor) == 0)
			need_ext_fw = 1;
	} else
		/* No way to check firmware version, reload to be sure */
		need_ext_fw = 1;

	if (need_ext_fw) {
		at76_dbg(DBG_DEVSTART, "downloading external firmware");

		ret = at76_load_external_fw(udev, fwe);
		if (ret)
			goto error;

		/* Re-check firmware version */
		ret = at76_get_mib(udev, MIB_FW_VERSION, &fwv, sizeof(fwv));
		if (ret < 0) {
			err("error %d getting firmware version", ret);
			goto error;
		}

		/* Major and minor version must match */
		if (fwv.major != fwe->fw_version.major
		    || fwv.minor != fwe->fw_version.minor) {
			printk(KERN_ERR DRIVER_NAME
			       ": wrong firmware version, loaded %d.%d.%d-%d, "
			       "read back %d.%d.%d-%d\n",
			       fwe->fw_version.major, fwe->fw_version.minor,
			       fwe->fw_version.patch, fwe->fw_version.build,
			       fwv.major, fwv.minor, fwv.patch, fwv.build);
			ret = -EBUSY;
			goto error;
		}
	}

	priv = at76_alloc_new_device(udev);
	if (!priv) {
		ret = -ENOMEM;
		goto error;
	}

	SET_NETDEV_DEV(priv->netdev, &interface->dev);
	usb_set_intfdata(interface, priv);

	memcpy(&priv->fw_version, &fwv, sizeof(struct mib_fw_version));
	priv->board_type = board_type;

	ret = at76_init_new_device(priv, interface);
	if (ret < 0)
		at76_delete_device(priv);

	return ret;

error:
	usb_put_dev(udev);
	return ret;
}

static void at76_disconnect(struct usb_interface *interface)
{
	struct at76_priv *priv;

	priv = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* Disconnect after loading internal firmware */
	if (!priv)
		return;

	printk(KERN_INFO "%s: disconnecting\n", priv->netdev->name);
	at76_delete_device(priv);
	printk(KERN_INFO DRIVER_NAME ": disconnected\n");
}

/* Structure for registering this driver with the USB subsystem */
static struct usb_driver at76_driver = {
	.name = DRIVER_NAME,
	.probe = at76_probe,
	.disconnect = at76_disconnect,
	.id_table = dev_table,
};

static int __init at76_mod_init(void)
{
	int result;

	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION " loading\n");

	mutex_init(&fw_mutex);

	/* register this driver with the USB subsystem */
	result = usb_register(&at76_driver);
	if (result < 0)
		err("usb_register failed (status %d)", result);

	led_trigger_register_simple("at76_usb-tx", &ledtrig_tx);
	return result;
}

static void __exit at76_mod_exit(void)
{
	int i;

	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION " unloading\n");
	usb_deregister(&at76_driver);
	for (i = 0; i < ARRAY_SIZE(firmwares); i++) {
		if (firmwares[i].fw)
			release_firmware(firmwares[i].fw);
	}
	led_trigger_unregister_simple(ledtrig_tx);
}

module_param_named(debug, at76_debug, int, 0600);
MODULE_PARM_DESC(debug, "Debugging level");

module_init(at76_mod_init);
module_exit(at76_mod_exit);

MODULE_AUTHOR("Oliver Kurth <oku@masqmail.cx>");
MODULE_AUTHOR("Joerg Albert <joerg.albert@gmx.de>");
MODULE_AUTHOR("Alex <alex@foogod.com>");
MODULE_AUTHOR("Nick Jones");
MODULE_AUTHOR("Balint Seeber <n0_5p4m_p13453@hotmail.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
