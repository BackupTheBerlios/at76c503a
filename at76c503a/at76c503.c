/* -*- linux-c -*- */
/*
 * USB at76c503/at76c505 driver
 *
 * Copyright (c) 2002 - 2003 Oliver Kurth <oku@masqmail.cx>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation; either version 2 of
 *	the License, or (at your option) any later version.
 *
 *
 * History:
 *
 * 2002_12_31:
 * - first ping, ah-hoc mode works, fw version 0.90.2 only
 *
 * 2003_01_07 0.1:
 * - first release
 *
 * 2003_01_08 0.2:
 * - moved rx code into tasklet
 * - added locking in keventd handler
 * - support big endian in ieee802_11.h
 * - external firmware downloader now in this module
 *
 * 2003_01_10 0.3:
 * - now using 8 spaces for tab indentations
 * - added tx rate settings (patch from Joerg Albert (jal))
 * - created functions for mib settings
 *
 * 2003_01_19 0.4:
 * - use usbdfu for the internal firmware
 *
 * 2003_01_27 0.5:
 * - implemented WEP. Thanks to jal
 * - added frag and rts ioctl calls (jal again)
 * - module parameter to give names other than eth
 *
 * 2003_01_28 0.6:
 * - make it compile with kernel < 2.4.20 (there is no owner field
 *   in struct usb_driver)
 * - fixed a small bug for the module param eth_name
 * - do not use GFP_DMA, GFP_KERNEL is enough
 * - no down() in _tx() because that's in interrupt. Use
 *   spin_lock_irq() instead
 * - should not stop net queue on urb errors
 * - cleanup in ioctl(): locked it altogether, this makes it easier
 *   to maintain
 * - tried to implement promisc. mode: does not work with this device
 * - tried to implement setting mac address: does not
 *   seem to work with this device
 * - now use fw version 0.90.2 #140 (prev. was #93). Does not help...
 *
 * 2003_01_30 0.7:
 * - now works with fw 0.100.2 (solution was: wait for completion
 *   of commands)
 * - setting MAC address now works (thx to a small fix by jal)
 * - it turned out that promisc. mode is not possible. The firmware
 *   does not allow it. I hope that it will be implemented in a future
 *   version.
 *
 * 2003_02_13 0.8:
 * - scan mode implemented by jal
 * - infra structure mode by jal
 * - some small cleanups (removed dead code)
 *
 * TODO:
 * - monitor mode
 * - should not need do drive device down, if changes are
 *   made with iwconfig
 * - scan for BSS
 * - infrastructure mode (if I can get an AP)
 * - fw 0.100.x
 * - optimize rx and tx (no memcpy)
 *
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/usb.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>

#include "at76c503.h"
#include "ieee802_11.h"

#ifdef CONFIG_USB_DEBUG
static int debug = 1;
#else
static int debug;
#endif

/* Use our own dbg macro */
#undef dbg
#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg); } while (0)

#ifndef min
#define min(x,y) ((x) < (y) ? (x) : (y))
#endif

#define assert(x) \
  do {\
   if (!(x)) \
     err(__FILE__ ":%d assertion " #x " failed", __LINE__);\
  } while (0)

/* how often do we re-try these packets ? */
#define AUTH_RETRIES  3
#define ASSOC_RETRIES 3
#define DISASSOC_RETRIES 3

#define NEW_STATE(dev,newstate) \
  do {\
    dbg("%s: state %d -> %d (" #newstate ")",\
        dev->netdev->name, dev->istate, newstate);\
    dev->istate = newstate;\
  } while (0)

/* the beacon timeout in infra mode when we are connected (in seconds) */
#define BEACON_TIMEOUT 10

/* after how many seconds do we re-scan the channels in infra mode */
#define RESCAN_TIME 10

/* Version Information */
#define DRIVER_DESC "Generic Atmel at76c503/at76c505 routines"

/* Module paramaters */
MODULE_PARM(debug, "i");
#define DRIVER_AUTHOR \
"Oliver Kurth <oku@masqmail.cx>, Joerg Albert <joerg.albert@gmx.de>, Alex <alex@foogod.com>"
MODULE_PARM_DESC(debug, "Debugging level");

static int rx_copybreak = 200;
MODULE_PARM(rx_copybreak, "i");
MODULE_PARM_DESC(rx_copybreak, "rx packet copy threshold");

struct header_struct {
        /* 802.3 */
        u8 dest[ETH_ALEN];
        u8 src[ETH_ALEN];
        u16 len;
        /* 802.2 */
        u8 dsap;
        u8 ssap;
        u8 ctrl;
        /* SNAP */
        u8 oui[3];
        u16 ethertype;
} __attribute__ ((packed));

#define DEF_RTS_THRESHOLD 1536
#define DEF_FRAG_THRESHOLD 1536
#define DEF_ESSID "okuwlan"
#define DEF_ESSID_LEN 7
#define DEF_CHANNEL 10

#define MAX_RTS_THRESHOLD 2347
#define MAX_FRAG_THRESHOLD 2346
#define MIN_FRAG_THRESHOLD 256

/* The frequency of each channel in MHz */
const long channel_frequency[] = {
        2412, 2417, 2422, 2427, 2432, 2437, 2442,
        2447, 2452, 2457, 2462, 2467, 2472, 2484
};
#define NUM_CHANNELS ( sizeof(channel_frequency) / sizeof(channel_frequency[0]) )

/* the broadcast address */
const u8 bc_addr[ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};

/* the supported rates of this hardware, bit7 marks a mandantory rate */
const u8 hw_rates[4] = {0x82,0x84,0x0b,0x16};

/* the max padding size for tx in bytes */
#define MAX_PADDING_SIZE 53

struct ieee802_11_mgmt {
	u16 frame_ctl;
	u16 duration_id;
	u8 addr1[ETH_ALEN]; /* destination addr */
	u8 addr2[ETH_ALEN]; /* source addr */
	u8 addr3[ETH_ALEN]; /* BSSID */
	u16 seq_ctl;
	u8 data[1508];
	u32 fcs;
} __attribute__ ((packed));

/* the size of the ieee802.11 header (incl. the at76c503 tx header) */
#define IEEE802_11_MGMT_HEADER_SIZE \
 (offsetof(struct at76c503_tx_buffer, packet) +\
  offsetof(struct ieee802_11_mgmt, data))

/* beacon in ieee802_11_mgmt.data */
struct ieee802_11_beacon_data {
	u8   timestamp[8];           // TSFTIMER
	u16  beacon_interval;         // Kms between TBTTs (Target Beacon Transmission Times)
	u16  capability_information;
	u8   data[1500]; /* contains: SSID (tag,length,value), 
			    Supported Rates (tlv), channel */
} __attribute__ ((packed));

/* disassoc frame in ieee802_11_mgmt.data */
struct ieee802_11_disassoc_frame {
	u16 reason;
} __attribute__ ((packed));
#define DISASSOC_FRAME_SIZE \
  (IEEE802_11_MGMT_HEADER_SIZE +\
   sizeof(struct ieee802_11_disassoc_frame))

/* assoc request in ieee802_11_mgmt.data */
struct ieee802_11_assoc_req {
	u16  capability;
	u16  listen_interval;
	u8   data[1]; /* variable number of bytes for SSID 
			 and supported rates (tlv coded) */
};
/* the maximum size of an AssocReq packet */
#define ASSOCREQ_MAX_SIZE \
  (IEEE802_11_MGMT_HEADER_SIZE +\
   offsetof(struct ieee802_11_assoc_req,data) +\
   1+1+IW_ESSID_MAX_SIZE + 1+1+4)

/* reassoc request in ieee802_11_mgmt.data */
struct ieee802_11_reassoc_req {
	u16  capability;
	u16  listen_interval;
	u8   curr_ap[ETH_ALEN]; /* the bssid of the AP we are
				   currently associated to */
	u8   data[1]; /* variable number of bytes for SSID 
			 and supported rates (tlv coded) */
} __attribute__ ((packed));

/* the maximum size of an AssocReq packet */
#define REASSOCREQ_MAX_SIZE \
  (IEEE802_11_MGMT_HEADER_SIZE +\
   offsetof(struct ieee802_11_reassoc_req,data) +\
   1+1+IW_ESSID_MAX_SIZE + 1+1+4)


/* assoc/reassoc response */
struct ieee802_11_assoc_resp {
	u16  capability;
	u16  status;
	u16  assoc_id;
	u8   data[1]; /* variable number of bytes for 
			 supported rates (tlv coded) */
} __attribute__ ((packed));

/* auth. request/response in ieee802_11_mgmt.data */
struct ieee802_11_auth_frame {
	u16 algorithm;
	u16 seq_nr;
	u16 status;
	/* no challenge text (yet) */
} __attribute__ ((packed));

/* deauth frame in ieee802_11_mgmt.data */
struct ieee802_11_deauth_frame {
	u16 reason;
} __attribute__ ((packed));
#define DEAUTH_FRAME_SIZE \
  (IEEE802_11_MGMT_HEADER_SIZE +\
   sizeof(struct ieee802_11_disauth_frame))

/* for shared key, add the challenge text size */
#define AUTH_FRAME_SIZE \
  (IEEE802_11_MGMT_HEADER_SIZE +\
   sizeof(struct ieee802_11_auth_frame))


#define KEVENT_CTRL_HALT 1
#define KEVENT_NEW_BSS 2
#define KEVENT_SET_PROMISC 3
#define KEVENT_MGMT_TIMEOUT 4
#define KEVENT_SCAN 5 
#define KEVENT_JOIN 6
#define KEVENT_STARTIBSS 7
#define KEVENT_SUBMIT_RX 8

static DECLARE_WAIT_QUEUE_HEAD(wait_queue);

static u8 snapsig[] = {0xaa, 0xaa, 0x03};
#ifdef COLLAPSE_RFC1042
/* RFC 1042 encapsulates Ethernet frames in 802.2 SNAP (0xaa, 0xaa, 0x03) with
 * a SNAP OID of 0 (0x00, 0x00, 0x00) */
static u8 rfc1042sig[] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00};
#endif /* COLLAPSE_RFC1042 */

/* local function prototypes */
	
static void at76c503_read_bulk_callback (struct urb *urb);
static void at76c503_write_bulk_callback(struct urb *urb);
static void defer_kevent (struct at76c503 *dev, int flag);
static int find_matching_bss(struct at76c503 *dev, int start);
static int auth_req(struct at76c503 *dev, int idx);
static int disassoc_req(struct at76c503 *dev, int idx);
static int assoc_req(struct at76c503 *dev, int idx);
static int reassoc_req(struct at76c503 *dev, int curr, int new);
static void dump_bss_table(struct at76c503 *dev);
static int submit_rx_urb(struct at76c503 *dev);

/* hexdump len many bytes from buf into obuf, separated by delim,
   add a trailing \0 into obuf */
static char *hex2str(char *obuf, u8 *buf, int len, char delim)
{
#define BIN2HEX(x) ((x) < 10 ? '0'+(x) : (x)+'A'-10)

  char *ret = obuf;
  while (len--) {
    *obuf++ = BIN2HEX(*buf>>4);
    *obuf++ = BIN2HEX(*buf&0xf);
    if (delim != '\0')
      *obuf++ = delim;
    buf++;
  }
  if (delim != '\0' && obuf > ret)
	  obuf--; // remove last inserted delimiter
  *obuf = '\0';

  return ret;
}

static inline char *mac2str(u8 *mac)
{
	static char str [6*3];
  
	sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return str;
}

static inline void usb_debug_data (const char *function, const unsigned char *data, int size)
{
	int i;

	if (!debug)
		return;
	
	printk (KERN_DEBUG __FILE__": %s - length = %d, data = ", 
		function, size);
	for (i = 0; i < size; ++i) {
		if((i % 8) == 0)
			printk ("\n");
		printk ("%.2x ", data[i]);
	}
	printk ("\n");
}

int at76c503_remap(struct usb_device *udev)
{
	int ret;
	ret = usb_control_msg(udev, usb_sndctrlpipe(udev,0),
			      0x0a, INTERFACE_VENDOR_REQUEST_OUT,
			      0, 0,
			      NULL, 0, HZ);
	if (ret < 0)
		return ret;

	return 0;
}

static inline
int get_op_mode(struct usb_device *udev)
{
	int ret;
	u8 op_mode;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			      0x33, INTERFACE_VENDOR_REQUEST_IN,
			      0x01, 0,
			      &op_mode, 1, HZ);
	if(ret < 0)
		return ret;
	return op_mode;
}

/* this loads a block of the second part of the firmware */
static inline
int load_ext_fw_block(struct usb_device *udev,
		      int i, unsigned char *buf, int bsize)
{
	return usb_control_msg(udev, usb_sndctrlpipe(udev,0),
			       0x0e, DEVICE_VENDOR_REQUEST_OUT,
			       0x0802, i,
			       buf, bsize, HZ);
}

static inline
int get_hw_cfg_rfmd(struct usb_device *udev,
	       unsigned char *buf, int buf_size)
{
	return usb_control_msg(udev, usb_rcvctrlpipe(udev,0),
			       0x33, INTERFACE_VENDOR_REQUEST_IN,
			       ((0x0a << 8) | 0x02), 0,
			       buf, buf_size, HZ);
}

/* Intersil boards use a different "value" for GetHWConfig requests */
static inline
int get_hw_cfg_intersil(struct usb_device *udev,
	       unsigned char *buf, int buf_size)
{
	return usb_control_msg(udev, usb_rcvctrlpipe(udev,0),
			       0x33, INTERFACE_VENDOR_REQUEST_IN,
			       ((0x09 << 8) | 0x02), 0,
			       buf, buf_size, HZ);
}

/* Get the hardware configuration for the adapter and place the appropriate
 * data in the appropriate fields of 'dev' (the GetHWConfig request and
 * interpretation of the result depends on the type of board we're dealing
 * with) */
static int get_hw_config(struct at76c503 *dev)
{
	int ret;
	union {
		struct hwcfg_intersil i;
		struct hwcfg_rfmd     r3;
		struct hwcfg_r505     r5;
	} *hwcfg = kmalloc(sizeof(*hwcfg), GFP_KERNEL);

	if (!hwcfg)
		return -ENOMEM;

	switch (dev->board_type) {
	  case BOARDTYPE_INTERSIL:
		ret = get_hw_cfg_intersil(dev->udev, (unsigned char *)&hwcfg->i, sizeof(hwcfg->i));
		if (ret < 0) break;
		memcpy(dev->mac_addr, hwcfg->i.mac_addr, ETH_ALEN);
		memcpy(dev->cr31_values, hwcfg->i.cr31_values, 14);
		memcpy(dev->cr58_values, hwcfg->i.cr58_values, 14);
		memcpy(dev->pidvid, hwcfg->i.pidvid, 4);
		dev->regulatory_domain = hwcfg->i.regulatory_domain;
		break;
	  case BOARDTYPE_RFMD:
		ret = get_hw_cfg_rfmd(dev->udev, (unsigned char *)&hwcfg->r3, sizeof(hwcfg->r3));
		if (ret < 0) break;
		memcpy(dev->cr20_values, hwcfg->r3.cr20_values, 14);
		memcpy(dev->cr21_values, hwcfg->r3.cr21_values, 14);
		memcpy(dev->bb_cr, hwcfg->r3.bb_cr, 14);
		memcpy(dev->pidvid, hwcfg->r3.pidvid, 4);
		memcpy(dev->mac_addr, hwcfg->r3.mac_addr, ETH_ALEN);
		dev->regulatory_domain = hwcfg->r3.regulatory_domain;
		memcpy(dev->low_power_values, hwcfg->r3.low_power_values, 14);
		memcpy(dev->normal_power_values, hwcfg->r3.normal_power_values, 14);
		break;
	  case BOARDTYPE_R505:
		ret = get_hw_cfg_rfmd(dev->udev, (unsigned char *)&hwcfg->r5, sizeof(hwcfg->r5));
		if (ret < 0) break;
		memcpy(dev->cr39_values, hwcfg->r5.cr39_values, 14);
		memcpy(dev->bb_cr, hwcfg->r5.bb_cr, 14);
		memcpy(dev->pidvid, hwcfg->r5.pidvid, 4);
		memcpy(dev->mac_addr, hwcfg->r5.mac_addr, ETH_ALEN);
		dev->regulatory_domain = hwcfg->r5.regulatory_domain;
		memcpy(dev->cr15_values, hwcfg->r5.cr15_values, 14);
		break;
	  default:
		err("Bad board type set (%d).  Unable to get hardware config.", dev->board_type);
		ret = -EINVAL;
	}

	kfree(hwcfg);

	if (ret < 0) {
		err("Get HW Config failed (%d)", ret);
	}
	return ret;
}

static inline
int get_mib(struct usb_device *udev,
	    u16 mib, u8 *buf, int buf_size)
{
	return usb_control_msg(udev, usb_rcvctrlpipe(udev,0),
			       0x33, INTERFACE_VENDOR_REQUEST_IN,
			       mib << 8, 0,
			       buf, buf_size, HZ);
}

static inline
int get_cmd_status(struct usb_device *udev,
		   u8 cmd, u8 *cmd_status)
{
	return usb_control_msg(udev, usb_rcvctrlpipe(udev,0),
			       0x22, INTERFACE_VENDOR_REQUEST_IN,
			       cmd, 0,
			       cmd_status, 40, HZ);
}

#define EXT_FW_BLOCK_SIZE 1024
int at76c503_download_external_fw(struct usb_device *udev, u8 *buf, int size)
{
	int i = 0, ret = 0;
	u8 op_mode;
	u8 *block;

	if (size < 0) return -EINVAL;
	if ((size > 0) && (buf == NULL)) return -EFAULT;

	op_mode = get_op_mode(udev);
	if (op_mode <= 0) {
		err("Internal firmware not loaded (%d)", op_mode);
		return -EPROTO;
	} 
	if (op_mode == OPMODE_NETCARD) {
		/* don't need firmware downloaded, it's already ready to go */
		return 0;
	}
	if (op_mode != OPMODE_NOFLASHNETCARD) {
		dbg("Unexpected operating mode (%d).  Attempting to download firmware anyway.", op_mode);
	}

	block = kmalloc(EXT_FW_BLOCK_SIZE, GFP_KERNEL);
	if (block == NULL) return -ENOMEM;

	info("Downloading external firmware...");

	while(size > 0){
		int bsize = size > EXT_FW_BLOCK_SIZE ? EXT_FW_BLOCK_SIZE : size;

		memcpy(block, buf, bsize);
		dbg("ext fw, size left = %5d, bsize = %4d, i = %2d", size, bsize, i);
		if((ret = load_ext_fw_block(udev, i, block, bsize)) < 0){
			err("load_ext_fw_block failed: %d, i = %d", ret, i);
			goto exit;
		}
		buf += bsize;
		size -= bsize;
		i++;
	}

	/* for fw >= 0.100, the device needs
	   an extra empty block: */
	if((ret = load_ext_fw_block(udev, i, block, 0)) < 0){
		err("load_ext_fw_block failed: %d, i = %d", ret, i);
		goto exit;
	}

 exit:
	kfree(block);
	return ret;
}

static
int set_card_command(struct usb_device *udev, int cmd,
		    unsigned char *buf, int buf_size)
{
	int ret;
	struct at76c503_command *cmd_buf =
		(struct at76c503_command *)kmalloc(
			sizeof(struct at76c503_command) + buf_size,
			GFP_KERNEL);

	if(cmd_buf){
		cmd_buf->cmd = cmd;
		cmd_buf->reserved = 0;
		cmd_buf->size = cpu_to_le16(buf_size);
		if(buf_size > 0)
			memcpy(&(cmd_buf[1]), buf, buf_size);
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev,0),
				      0x0e, DEVICE_VENDOR_REQUEST_OUT,
				      0, 0,
				      cmd_buf,
				      sizeof(struct at76c503_command) + buf_size,
				      HZ);
		kfree(cmd_buf);
		return ret;
	}

	return -ENOMEM;
}

/* TODO: should timeout */
int wait_completion(struct at76c503 *dev, int cmd)
{
	u8 *cmd_status = kmalloc(40, GFP_KERNEL);
	struct net_device *netdev = dev->netdev;
	int ret = 0;

	do{
		ret = get_cmd_status(dev->udev, cmd, cmd_status);
		if(ret < 0){
			err("%s: get_cmd_status failed: %d", netdev->name, ret);
			break;
		}
#if 0
		info("cmd %d,cmd_status[5] = %d", cmd,cmd_status[5]);
#endif

		if(cmd_status[5] == CMD_STATUS_IN_PROGRESS ||
		   cmd_status[5] == CMD_STATUS_IDLE){
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(HZ/10); // 100 ms
		}else break;
	}while(1);
	if (ret >= 0)
		/* if get_cmd_status did not fail, return the status
		   retrieved */
		ret = cmd_status[5];
	kfree(cmd_status);
	return ret;
}

static inline
int set_mib(struct usb_device *udev, struct set_mib_buffer *buf)
{
	int ret;
	struct at76c503_command *cmd_buf =
		(struct at76c503_command *)kmalloc(
			sizeof(struct at76c503_command) + buf->size + 4,
			GFP_KERNEL);

	if(cmd_buf){
		cmd_buf->cmd = CMD_SET_MIB;
		cmd_buf->reserved = 0;
		cmd_buf->size = cpu_to_le16(buf->size + 4);
		memcpy(&(cmd_buf[1]), buf, buf->size + 4);
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev,0),
				      0x0e, DEVICE_VENDOR_REQUEST_OUT,
				      0, 0,
				      cmd_buf,
				      sizeof(struct at76c503_command) + buf->size + 4,
				      HZ);
		kfree(cmd_buf);
		return ret;
	}

	return -ENOMEM;
}

static
int set_radio(struct at76c503 *dev, int on_off)
{
	int ret = 0;

	if(dev->radio_on != on_off){
		ret = set_card_command(dev->udev, CMD_RADIO, NULL, 0);
		if(ret < 0){
			err("%s: set_card_command(CMD_RADIO) failed: %d", dev->netdev->name, ret);
		}
		dev->radio_on = on_off;
	}
	return ret;
}


static
int set_preamble(struct at76c503 *dev, u8 type)
{
	int ret = 0;
	struct set_mib_buffer mib_buf;

	memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
	mib_buf.type = MIB_LOCAL;
	mib_buf.size = 1;
	mib_buf.index = PREAMBLE_TYPE_OFFSET;
	mib_buf.data[0] = type;
	ret = set_mib(dev->udev, &mib_buf);
	if(ret < 0){
		err("%s: set_mib (preamble) failed: %d", dev->netdev->name, ret);
	}
	return ret;
}

static
int set_frag(struct at76c503 *dev, u16 size)
{
	int ret = 0;
	struct set_mib_buffer mib_buf;

	memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
	mib_buf.type = MIB_MAC;
	mib_buf.size = 2;
	mib_buf.index = FRAGMENTATION_OFFSET;
	*(u16*)mib_buf.data = cpu_to_le16(size);
	ret = set_mib(dev->udev, &mib_buf);
	if(ret < 0){
		err("%s: set_mib (frag threshold) failed: %d", dev->netdev->name, ret);
	}
	return ret;
}

static
int set_rts(struct at76c503 *dev, u16 size)
{
	int ret = 0;
	struct set_mib_buffer mib_buf;

	memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
	mib_buf.type = MIB_MAC;
	mib_buf.size = 2;
	mib_buf.index = RTS_OFFSET;
	*(u16*)mib_buf.data = cpu_to_le16(size);
	ret = set_mib(dev->udev, &mib_buf);
	if(ret < 0){
		err("%s: set_mib (rts) failed: %d", dev->netdev->name, ret);
	}
	return ret;
}

static
int set_autorate_fallback(struct at76c503 *dev, int onoff)
{
	int ret = 0;
	struct set_mib_buffer mib_buf;

	memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
	mib_buf.type = MIB_LOCAL;
	mib_buf.size = 1;
	mib_buf.index = TX_AUTORATE_FALLBACK_OFFSET;
	mib_buf.data[0] = onoff;
	ret = set_mib(dev->udev, &mib_buf);
	if(ret < 0){
		err("%s: set_mib (autorate fallback) failed: %d", dev->netdev->name, ret);
	}
	return ret;
}

static int set_mac_address(struct at76c503 *dev, void *addr)
{
        struct set_mib_buffer mib_buf;
        int ret = 0;

        memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
        mib_buf.type = MIB_MAC_ADD;
        mib_buf.size = ETH_ALEN;
        mib_buf.index = offsetof(struct mib_mac_addr, mac_addr);
        memcpy(mib_buf.data, addr, ETH_ALEN);
        ret = set_mib(dev->udev, &mib_buf);
        if(ret < 0){
                err("%s: set_mib (MAC_ADDR, mac_addr) failed: %d",
                    dev->netdev->name, ret);
        }
        return ret;
}

#if 0
/* implemented to get promisc. mode working, but does not help.
   May still be useful for multicast eventually. */
static int set_group_address(struct at76c503 *dev, u8 *addr, int n)
{
        struct set_mib_buffer mib_buf;
        int ret = 0;

        memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
        mib_buf.type = MIB_MAC_ADD;
        mib_buf.size = ETH_ALEN;
        mib_buf.index = offsetof(struct mib_mac_addr, group_addr) + n*ETH_ALEN;
        memcpy(mib_buf.data, addr, ETH_ALEN);
        ret = set_mib(dev->udev, &mib_buf);
        if(ret < 0){
                err("%s: set_mib (MIB_MAC_ADD, group_addr) failed: %d",
                    dev->netdev->name, ret);
        }

#if 1
	/* I do not know anything about the group_addr_status field... (oku)*/
	wait_completion(dev, CMD_SET_MIB);

        memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
        mib_buf.type = MIB_MAC_ADD;
        mib_buf.size = 1;
        mib_buf.index = offsetof(struct mib_mac_addr, group_addr_status) + n;
        mib_buf.data[0] = 1;
        ret = set_mib(dev->udev, &mib_buf);
        if(ret < 0){
                err("%s: set_mib (MIB_MAC_ADD, group_addr_status) failed: %d",
                    dev->netdev->name, ret);
        }
#endif
        return ret;
}
#endif

static
int set_promisc(struct at76c503 *dev, int onoff)
{
	int ret = 0;
	struct set_mib_buffer mib_buf;

	memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
	mib_buf.type = MIB_LOCAL;
	mib_buf.size = 1;
	mib_buf.index = offsetof(struct mib_local, promiscuous_mode);
	mib_buf.data[0] = onoff ? 1 : 0;
	ret = set_mib(dev->udev, &mib_buf);
	if(ret < 0){
		err("%s: set_mib (promiscous_mode) failed: %d", dev->netdev->name, ret);
	}
	return ret;
}

static
int get_current_bssid(struct at76c503 *dev)
{
	int ret = 0;
	struct mib_mac_mgmt *mac_mgmt =
		kmalloc(sizeof(struct mib_mac_mgmt), GFP_KERNEL);

	if(!mac_mgmt){
		ret = -ENOMEM;
		goto exit;
	}
	
	ret = get_mib(dev->udev, MIB_MAC_MGMT,
		      (u8*)mac_mgmt, sizeof(struct mib_mac_mgmt));
	if(ret < 0){
		err("%s: get_mib failed: %d", dev->netdev->name, ret);
		goto err;
	}
	memcpy(dev->bssid, mac_mgmt->current_bssid, ETH_ALEN);
	info("using BSSID %s", mac2str(dev->bssid));
 err:
	kfree(mac_mgmt);
 exit:
	return ret;
}

static
int get_current_channel(struct at76c503 *dev)
{
	int ret = 0;
	struct mib_phy *phy =
		kmalloc(sizeof(struct mib_phy), GFP_KERNEL);

	if(!phy){
		ret = -ENOMEM;
		goto exit;
	}
	ret = get_mib(dev->udev, MIB_PHY, (u8*)phy,
		      sizeof(struct mib_phy));
	if(ret < 0){
		err("%s: get_mib(MIB_PHY) failed: %d", dev->netdev->name, ret);
		goto err;
	}
	dev->channel = phy->channel_id;
 err:
	kfree(phy);
 exit:
	return ret;
}

static
int start_scan(struct at76c503 *dev)
{
	struct at76c503_start_scan scan;

	dev->bss_nr = 0; /* empty the table dev->bss[] */
	dev->curr_bss = dev->new_bss = -1;

	memset(&scan, 0, sizeof(struct at76c503_start_scan));
	memset(scan.bssid, 0xff, ETH_ALEN);
	//jal: we scan for all SSID and choose the right one later
	//     to see our env etc.
	//memcpy(scan.essid, dev->essid, IW_ESSID_MAX_SIZE);
	scan.essid_size = 0;
	scan.probe_delay = 10000;
	//jal: why should we start at a certain channel? we do scan the whole range
	//allowed by reg domain.
	scan.channel = dev->channel;

	/* atmelwlandriver differs between scan type 0 and 1.
	   For ad-hoc mode, it uses type 0 only.*/

	scan.min_channel_time = 10;
	scan.max_channel_time = 120;
	/* other values are set to 0 for type 0 */

	return set_card_command(dev->udev, CMD_SCAN,
				(unsigned char*)&scan, sizeof(scan));
}

static
int start_ibss(struct at76c503 *dev)
{
	struct at76c503_start_bss bss;

	memset(&bss, 0, sizeof(struct at76c503_start_bss));
	memset(bss.bssid, 0xff, ETH_ALEN);
	memcpy(bss.essid, dev->essid, IW_ESSID_MAX_SIZE);
	bss.essid_size = dev->essid_size;
	bss.bss_type = ADHOC_MODE;
	bss.channel = dev->channel;

	return set_card_command(dev->udev, CMD_START_IBSS,
				(unsigned char*)&bss, sizeof(struct at76c503_start_bss));
}

/* idx points into dev->bss */
static
int join_bss(struct at76c503 *dev, int idx)
{
	struct at76c503_join join;
	struct bss_info *ptr;

	assert(idx < dev->bss_nr);
	ptr = dev->bss+idx;

	memset(&join, 0, sizeof(struct at76c503_join));
	memcpy(join.bssid, ptr->bssid, ETH_ALEN);
	memcpy(join.essid, ptr->ssid, ptr->ssid_len);
	join.essid_size = ptr->ssid_len;
	join.bss_type = (dev->iw_mode == IW_MODE_ADHOC ? 1 : 2);
	join.channel = ptr->channel;
	join.timeout = 2000;

	dbg("%s join addr %s ssid %d:%s type %d ch %d timeout %d",
	    dev->netdev->name, mac2str(join.bssid), 
	    join.essid_size, join.essid,
	    join.bss_type, join.channel, join.timeout);
	return set_card_command(dev->udev, CMD_JOIN,
				(unsigned char*)&join,
				sizeof(struct at76c503_join));
} /* join_bss */

/* we got a timeout for a infrastructure mgmt packet */
void mgmt_timeout(unsigned long par)
{
	struct at76c503 *dev = (struct at76c503 *)par;
	defer_kevent(dev, KEVENT_MGMT_TIMEOUT);
}

/* the deferred procedure called from kevent() */
void handle_mgmt_timeout(struct at76c503 *dev)
{

	dbg("%s: timeout, state %d", dev->netdev->name, dev->istate);

	switch(dev->istate) {

	case SCANNING: /* we use the mgmt_timer to delay the next scan for some time */
		defer_kevent(dev, KEVENT_SCAN);
		break;

	case JOINING:
		assert(0);
		break;

	case CONNECTED: /* we haven't received the beacon of this BSS for 
			   BEACON_TIMEOUT seconds */
		info("%s: lost beacon bssid %s",
		     dev->netdev->name, mac2str(dev->bss[dev->curr_bss].bssid));
		netif_carrier_off(dev->netdev); /* disable running netdev watchdog */
		netif_stop_queue(dev->netdev); /* stop tx data packets */
		NEW_STATE(dev,SCANNING);
		defer_kevent(dev,KEVENT_SCAN);
		break;

	case AUTHENTICATING:
		if (dev->retries-- >= 0) {
			auth_req(dev,dev->curr_bss);
			mod_timer(&dev->mgmt_timer, jiffies+HZ);
		} else {
			/* try to get next matching BSS */
			NEW_STATE(dev,JOINING);
			defer_kevent(dev,KEVENT_JOIN);
		}
		break;

	case ASSOCIATING:
		if (dev->retries-- >= 0) {
			assoc_req(dev,dev->curr_bss);
			mod_timer(&dev->mgmt_timer, jiffies+HZ);
		} else {
			/* jal: TODO: we may be authenticated to several
			   BSS and may try to associate to the next of them here
			   in the future ... */

			/* try to get next matching BSS */
			NEW_STATE(dev,JOINING);
			defer_kevent(dev,KEVENT_JOIN);
		}
		break;

	case REASSOCIATING:
		if (dev->retries-- >= 0)
			reassoc_req(dev, dev->curr_bss, dev->new_bss);
		else {
			/* we disassociate from the curr_bss and
			   scan again ... */
			NEW_STATE(dev,DISASSOCIATING);
			dev->retries = DISASSOC_RETRIES;
			disassoc_req(dev, dev->curr_bss);
		}
		mod_timer(&dev->mgmt_timer, jiffies+HZ);
		break;

	case DISASSOCIATING:
		if (dev->retries-- >= 0) {
			disassoc_req(dev, dev->curr_bss);
			mod_timer(&dev->mgmt_timer,jiffies+HZ);
		} else {
			/* we scan again ... */
			NEW_STATE(dev,SCANNING);
			defer_kevent(dev,KEVENT_SCAN);
		}
		break;

	case INIT:
		assert(0);
		break;

	default:
		assert(0);
	} /* switch (dev->istate) */

}/* handle_mgmt_timeout */

/* calc. the padding from txbuf->wlength (which excludes the USB TX header) */
static inline
int calc_padding(int wlen)
{
	wlen += offsetof(struct at76c503_tx_buffer,packet); /* add the USB TX header */

	wlen = wlen % 64;

	if (wlen < 50)
		return 50 - wlen;

	if (wlen >=61)
		return 114 - wlen;

	return 0;
}

/* send a management frame on bulk-out */
static
int send_mgmt_bulk(struct at76c503 *dev, struct at76c503_tx_buffer *txbuf)
{
	unsigned long flags;
	char obuf[3*64+1] __attribute__ ((unused));
	int ret = 0;
	int urb_status;
	void *oldbuf = NULL;

	netif_carrier_off(dev->netdev); /* disable running netdev watchdog */
	netif_stop_queue(dev->netdev); /* stop tx data packets */

	spin_lock_irqsave(&dev->mgmt_spinlock, flags);

	if ((urb_status=dev->write_urb->status) == USB_ST_URB_PENDING) {
		oldbuf=dev->next_mgmt_bulk; /* to kfree below */
		dev->next_mgmt_bulk = txbuf;
		txbuf = NULL;
	}
	spin_unlock_irqrestore(&dev->mgmt_spinlock, flags);

	if (oldbuf) {
		/* a data/mgmt tx is already pending in the URB -
		   if this is no error in some situations we must
		   implement a queue or silently modify the old msg */
		err("%s: %s removed pending mgmt buffer %s",
		    dev->netdev->name, __FUNCTION__,
		    hex2str(obuf, (u8 *)dev->next_mgmt_bulk, 64,' '));
		kfree(dev->next_mgmt_bulk);
	}

	if (txbuf) {

		txbuf->tx_rate = 0;
		txbuf->padding = 0;
//		txbuf->padding = 
//		   cpu_to_le16(calc_padding(le16_to_cpu(txbuf->wlength)));
		if (dev->next_mgmt_bulk) {
			err("%s: %s URB status %d, but mgmt is pending",
			    dev->netdev->name, __FUNCTION__, urb_status);
		}
#if 0
		dbg("%s: tx mgmt: wlen %d tx_rate %d pad %d %s",
		    dev->netdev->name, le16_to_cpu(txbuf->wlength),
		    txbuf->tx_rate, txbuf->padding,
		    hex2str(obuf,txbuf->packet,
			    min((sizeof(obuf)-1)/2,
				(size_t)le16_to_cpu(txbuf->wlength)),'\0'));
#endif
		/* txbuf was not consumed above -> send mgmt msg immediately */
		memcpy(dev->bulk_out_buffer, txbuf,
		       le16_to_cpu(txbuf->wlength) +
		       offsetof(struct at76c503_tx_buffer,packet));
		FILL_BULK_URB(dev->write_urb, dev->udev,
			      usb_sndbulkpipe(dev->udev, 
					      dev->bulk_out_endpointAddr),
			      dev->bulk_out_buffer,
			      le16_to_cpu(txbuf->wlength) + 
			      le16_to_cpu(txbuf->padding) +
			      offsetof(struct at76c503_tx_buffer,packet),
			      at76c503_write_bulk_callback, dev);
		ret = usb_submit_urb(dev->write_urb);
		if (ret) {
			err("%s: %s error in tx submit urb: %d",
			    dev->netdev->name, __FUNCTION__, ret);
		}
		kfree(txbuf);
	} /* if (txbuf) */

	return ret;

} /* send_mgmt_bulk */

static
int disassoc_req(struct at76c503 *dev, int idx)
{
	struct at76c503_tx_buffer *tx_buffer;
	struct ieee802_11_mgmt *mgmt;
	struct ieee802_11_disassoc_frame *req;
	struct bss_info *bss = dev->bss+idx;

	assert(idx >= 0 && idx < dev->bss_nr);

	tx_buffer = kmalloc(DISASSOC_FRAME_SIZE, GFP_ATOMIC);
	if (!tx_buffer)
		return -ENOMEM;

	mgmt = (struct ieee802_11_mgmt *)&(tx_buffer->packet);
	req  = (struct ieee802_11_disassoc_frame *)&(mgmt->data);

	/* make wireless header */
	/* no need to care about endianness of constants - is taken care
	   of in ieee802_11.h */
	mgmt->frame_ctl = IEEE802_11_FTYPE_MGMT|IEEE802_11_STYPE_AUTH;
	mgmt->duration_id = cpu_to_le16(0x8000);
	memcpy(mgmt->addr1, bss->bssid, ETH_ALEN);
	memcpy(mgmt->addr2, dev->netdev->dev_addr, ETH_ALEN);
	memcpy(mgmt->addr3, bss->bssid, ETH_ALEN);
	mgmt->seq_ctl = 0;

	req->reason = 0;

	/* init. at76c503 tx header */
	tx_buffer->wlength = cpu_to_le16(DISASSOC_FRAME_SIZE -
		offsetof(struct at76c503_tx_buffer, packet));
	
	dbg("%s: DisAssocReq bssid %s",
	    dev->netdev->name, mac2str(mgmt->addr3));

	/* either send immediately (if no data tx is pending
	   or put it in pending list */
	return send_mgmt_bulk(dev, tx_buffer); 

} /* disassoc_req */

static
int auth_req(struct at76c503 *dev, int idx)
{
	struct at76c503_tx_buffer *tx_buffer;
	struct ieee802_11_mgmt *mgmt;
	struct ieee802_11_auth_frame *req;
	struct bss_info *bss = dev->bss+idx;

	assert(idx >= 0 && idx < dev->bss_nr);

	tx_buffer = kmalloc(AUTH_FRAME_SIZE, GFP_ATOMIC);
	if (!tx_buffer)
		return -ENOMEM;

	mgmt = (struct ieee802_11_mgmt *)&(tx_buffer->packet);
	req  = (struct ieee802_11_auth_frame *)&(mgmt->data);

	/* make wireless header */
	/* no need to care about endianness of constants - is taken care
	   of in ieee802_11.h */
	/* first auth msg is not encrypted */
	mgmt->frame_ctl = IEEE802_11_FTYPE_MGMT|IEEE802_11_STYPE_AUTH;
	mgmt->duration_id = cpu_to_le16(0x8000);
	memcpy(mgmt->addr1, bss->bssid, ETH_ALEN);
	memcpy(mgmt->addr2, dev->netdev->dev_addr, ETH_ALEN);
	memcpy(mgmt->addr3, bss->bssid, ETH_ALEN);
	mgmt->seq_ctl = 0;

	req->algorithm = IEEE802_11_AUTH_ALG_OPEN_SYSTEM;
	req->seq_nr = cpu_to_le16(1);
	req->status = 0;

	/* init. at76c503 tx header */
	tx_buffer->wlength = cpu_to_le16(AUTH_FRAME_SIZE -
		offsetof(struct at76c503_tx_buffer, packet));
	
	dbg("%s: AuthReq bssid %s algorithm %d trans_seq %d",
	    dev->netdev->name, mac2str(mgmt->addr3),
	    req->algorithm, req->seq_nr);

	/* either send immediately (if no data tx is pending
	   or put it in pending list */
	return send_mgmt_bulk(dev, tx_buffer); 

} /* auth_req */

static
int assoc_req(struct at76c503 *dev, int idx)
{
	struct at76c503_tx_buffer *tx_buffer;
	struct ieee802_11_mgmt *mgmt;
	struct ieee802_11_assoc_req *req;
	struct bss_info *bss = dev->bss+idx;
	u8 *tlv;

	assert(idx >= 0 && idx < dev->bss_nr);

	tx_buffer = kmalloc(ASSOCREQ_MAX_SIZE, GFP_ATOMIC);
	if (!tx_buffer)
		return -ENOMEM;

	mgmt = (struct ieee802_11_mgmt *)&(tx_buffer->packet);
	req  = (struct ieee802_11_assoc_req *)&(mgmt->data);
	tlv = req->data;

	/* make wireless header */
	/* no need to care about endianness of constants - is taken care
	   of in ieee802_11.h */
	mgmt->frame_ctl = IEEE802_11_FTYPE_MGMT|IEEE802_11_STYPE_ASSOC_REQ;

	mgmt->duration_id = cpu_to_le16(0x8000);
	memcpy(mgmt->addr1, bss->bssid, ETH_ALEN);
	memcpy(mgmt->addr2, dev->netdev->dev_addr, ETH_ALEN);
	memcpy(mgmt->addr3, bss->bssid, ETH_ALEN);
	mgmt->seq_ctl = 0;

	req->capability = bss->capa;

	/* we must set the Privacy bit in the capabilites to assure an
	   Agere-based AP with optional WEP transmits encrypted frames
	   to us.  AP only set the Privacy bit in their capabilities
	   if WEP is mandatory in the BSS! */
	if (dev->wep_enabled)
		req->capability |= IEEE802_11_CAPA_PRIVACY;
	if (dev->preamble_type == PREAMBLE_TYPE_SHORT)
		req->capability |= IEEE802_11_CAPA_SHORT_PREAMBLE;
		
	req->listen_interval = cpu_to_le16(2 * bss->beacon_interval);
	
	/* write TLV data elements */

	*tlv++ = IE_ID_SSID;
	*tlv++ = bss->ssid_len;
	memcpy(tlv,bss->ssid, bss->ssid_len);
	tlv += bss->ssid_len;

	*tlv++ = IE_ID_SUPPORTED_RATES;
	*tlv++ = sizeof(hw_rates);
	memcpy(tlv, hw_rates, sizeof(hw_rates));
	tlv += sizeof(hw_rates); /* tlv points behind the supp_rates field */

	/* init. at76c503 tx header */
	tx_buffer->wlength = cpu_to_le16(tlv-(u8 *)mgmt);
	
	{
		/* output buffer for ssid and rates */
		char ossid[IW_ESSID_MAX_SIZE+1] __attribute__ ((unused));
		char orates[4*2+1] __attribute__ ((unused));
		int len;

		tlv = req->data;
		len = min(sizeof(ossid)-1,(size_t)*(tlv+1));
		memcpy(ossid, tlv+2, len);
		ossid[len] = '\0';
		tlv += (1 + 1 + *(tlv+1)); /* points to IE of rates now */
		dbg("%s: AssocReq bssid %s capa x%04x ssid %s rates %s",
		    dev->netdev->name, mac2str(mgmt->addr3),
		    req->capability, ossid,
		    hex2str(orates,tlv+2,min((sizeof(orates)-1)/2,(size_t)*(tlv+1)),
			    '\0'));
	}

	/* either send immediately (if no data tx is pending
	   or put it in pending list */
	return send_mgmt_bulk(dev, tx_buffer); 

} /* assoc_req */

/* we are currently associated to dev->bss[curr] and
   want to reassoc to dev->bss[idx] */
static
int reassoc_req(struct at76c503 *dev, int curr, int idx)
{
	struct at76c503_tx_buffer *tx_buffer;
	struct ieee802_11_mgmt *mgmt;
	struct ieee802_11_reassoc_req *req;
	struct bss_info *bss = dev->bss+idx;
	u8 *tlv;

	assert(curr >= 0 && curr < dev->bss_nr);
	assert(idx >= 0 && idx < dev->bss_nr);

	tx_buffer = kmalloc(REASSOCREQ_MAX_SIZE, GFP_ATOMIC);
	if (!tx_buffer)
		return -ENOMEM;

	mgmt = (struct ieee802_11_mgmt *)&(tx_buffer->packet);
	req  = (struct ieee802_11_reassoc_req *)&(mgmt->data);
	tlv = req->data;

	/* make wireless header */
	/* no need to care about endianness of constants - is taken care
	   of in ieee802_11.h */
	/* jal: encrypt this packet if wep_enabled is TRUE ??? */
	mgmt->frame_ctl = IEEE802_11_FTYPE_MGMT|IEEE802_11_STYPE_REASSOC_REQ;
	mgmt->duration_id = cpu_to_le16(0x8000);
	memcpy(mgmt->addr1, bss->bssid, ETH_ALEN);
	memcpy(mgmt->addr2, dev->netdev->dev_addr, ETH_ALEN);
	memcpy(mgmt->addr3, bss->bssid, ETH_ALEN);
	mgmt->seq_ctl = 0;

	req->capability = bss->capa;

	/* we must set the Privacy bit in the capabilites to assure an
	   Agere-based AP with optional WEP transmits encrypted frames
	   to us.  AP only set the Privacy bit in their capabilities
	   if WEP is mandatory in the BSS! */
	if (dev->wep_enabled)
		req->capability |= IEEE802_11_CAPA_PRIVACY;
	if (dev->preamble_type == PREAMBLE_TYPE_SHORT)
		req->capability |= IEEE802_11_CAPA_SHORT_PREAMBLE;
		
	req->listen_interval = cpu_to_le16(2 * bss->beacon_interval);
	
	memcpy(req->curr_ap, dev->bss[curr].bssid, ETH_ALEN);

	/* write TLV data elements */

	*tlv++ = IE_ID_SSID;
	*tlv++ = bss->ssid_len;
	memcpy(tlv,bss->ssid, bss->ssid_len);
	tlv += bss->ssid_len;

	*tlv++ = IE_ID_SUPPORTED_RATES;
	*tlv++ = sizeof(hw_rates);
	memcpy(tlv, hw_rates, sizeof(hw_rates));
	tlv += sizeof(hw_rates); /* tlv points behind the supp_rates field */

	/* init. at76c503 tx header */
	tx_buffer->wlength = cpu_to_le16(tlv-(u8 *)mgmt);
	
	{
		/* output buffer for ssid and rates */
		char ossid[IW_ESSID_MAX_SIZE+1] __attribute__ ((unused));
		char orates[4*2+1] __attribute__ ((unused));
		char ocurr[6*3+1] __attribute__ ((unused));
		tlv = req->data;
		memcpy(ossid, tlv+2, min(sizeof(ossid),(size_t)*(tlv+1)));
		ossid[sizeof(ossid)-1] = '\0';
		tlv += (1 + 1 + *(tlv+1)); /* points to IE of rates now */
		dbg("%s: ReAssocReq curr %s new %s capa x%04x ssid %s rates %s",
		    dev->netdev->name,
		    hex2str(ocurr, req->curr_ap, ETH_ALEN, ':'),
		    mac2str(mgmt->addr3), req->capability, ossid,
		    hex2str(orates,tlv+2,min((sizeof(orates)-1)/2,(size_t)*(tlv+1)),
			    '\0'));
	}

	/* either send immediately (if no data tx is pending
	   or put it in pending list */
	return send_mgmt_bulk(dev, tx_buffer); 

} /* reassoc_req */


/* shamelessly copied from usbnet.c (oku) */
static void defer_kevent (struct at76c503 *dev, int flag)
{
	set_bit (flag, &dev->kevent_flags);
	if (!schedule_task (&dev->kevent))
		err ("%s: kevent %d may have been dropped",
		     dev->netdev->name, flag);
	else
		dbg ("%s: kevent %d scheduled", dev->netdev->name, flag);
}

static void
kevent(void *data)
{
	struct at76c503 *dev = data;
	int ret;

	/* on errors, bits aren't cleared, but no reschedule
	   is done. So work will be done next time something
	   else has to be done. This is ugly. TODO! (oku) */

	dbg("%s: kevent flags=x%x", dev->netdev->name, dev->kevent_flags);

	down(&dev->sem);

	if(test_bit(KEVENT_CTRL_HALT, &dev->kevent_flags)){
		/* this never worked... but it seems
		   that it's rarely necessary, if at all (oku) */
		ret = usb_clear_halt(dev->udev,
				     usb_sndctrlpipe (dev->udev, 0));
		if(ret < 0)
			err("usb_clear_halt() failed: %d", ret);
		else{
			clear_bit(KEVENT_CTRL_HALT, &dev->kevent_flags);
			info("usb_clear_halt() succesful");
		}
	}
	if(test_bit(KEVENT_NEW_BSS, &dev->kevent_flags)){
		struct net_device *netdev = dev->netdev;
		struct mib_mac_mgmt *mac_mgmt = kmalloc(sizeof(struct mib_mac_mgmt), GFP_KERNEL);
		struct set_mib_buffer mib_buf;

		ret = get_mib(dev->udev, MIB_MAC_MGMT, (u8*)mac_mgmt,
			      sizeof(struct mib_mac_mgmt));
		if(ret < 0){
			err("%s: get_mib failed: %d", netdev->name, ret);
			goto new_bss_clean;
		}
		//    usb_debug_data(__FUNCTION__, (unsigned char *)mac_mgmt, sizeof(struct mib_mac_mgmt));


		dbg("ibss_change = 0x%2x", mac_mgmt->ibss_change);
		memcpy(dev->bssid, mac_mgmt->current_bssid, ETH_ALEN);
		info("using BSSID %s", mac2str(dev->bssid));
    
		memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
		mib_buf.type = MIB_MAC_MGMT;
		mib_buf.size = 1;
		mib_buf.index = IBSS_CHANGE_OK_OFFSET;
		ret = set_mib(dev->udev, &mib_buf);
		if(ret < 0){
			err("%s: set_mib (ibss change ok) failed: %d", netdev->name, ret);
			goto new_bss_clean;
		}

		wait_completion(dev, CMD_SET_MIB);

		clear_bit(KEVENT_NEW_BSS, &dev->kevent_flags);
	new_bss_clean:
		kfree(mac_mgmt);
	}
	if(test_bit(KEVENT_SET_PROMISC, &dev->kevent_flags)){
		info("%s: KEVENT_SET_PROMISC", dev->netdev->name);

		set_promisc(dev, dev->promisc);
		wait_completion(dev, CMD_SET_MIB);
		clear_bit(KEVENT_SET_PROMISC, &dev->kevent_flags);
	}

	if(test_bit(KEVENT_MGMT_TIMEOUT, &dev->kevent_flags)){
		clear_bit(KEVENT_MGMT_TIMEOUT, &dev->kevent_flags);
		handle_mgmt_timeout(dev);
	}

	/* check this _before_ KEVENT_JOIN, 'cause _JOIN sets _STARTIBSS bit */
	if (test_bit(KEVENT_STARTIBSS, &dev->kevent_flags)) {
		struct set_mib_buffer mib_buf;
		clear_bit(KEVENT_STARTIBSS, &dev->kevent_flags);
		assert(dev->istate == STARTIBSS);
		ret = start_ibss(dev);
		if(ret < 0){
			err("%s: start_ibss failed: %d", dev->netdev->name, ret);
			goto end_startibss;
		}

		ret = wait_completion(dev, CMD_START_IBSS);
		if (ret != CMD_STATUS_COMPLETE) {
			err("%s start_ibss failed to complete,%d",
			    dev->netdev->name, ret);
			goto end_startibss;
		}

		ret = get_current_bssid(dev);
		if(ret < 0) goto end_startibss;

		ret = get_current_channel(dev);
		if(ret < 0) goto end_startibss;

		/* not sure what this is good for ??? */
		memset(&mib_buf, 0, sizeof(struct set_mib_buffer));
		mib_buf.type = MIB_MAC_MGMT;
		mib_buf.size = 1;
		mib_buf.index = IBSS_CHANGE_OK_OFFSET;
		ret = set_mib(dev->udev, &mib_buf);
		if(ret < 0){
			err("%s: set_mib (ibss change ok) failed: %d", dev->netdev->name, ret);
			goto end_startibss;
		}

		netif_start_queue(dev->netdev);
	}
end_startibss:

	/* check this _before_ KEVENT_SCAN, 'cause _SCAN sets _JOIN bit */
	if (test_bit(KEVENT_JOIN, &dev->kevent_flags)) {
		clear_bit(KEVENT_JOIN, &dev->kevent_flags);
		assert(dev->istate == JOINING);
		if ((dev->curr_bss=find_matching_bss(dev,dev->curr_bss+1)) >= 0) {
			if ((ret=join_bss(dev,dev->curr_bss)) < 0) {
				err("%s: join_bss failed with %d",
				    dev->netdev->name, ret);
				goto end_join;
			}
			
			ret=wait_completion(dev,CMD_JOIN);
			if (ret != CMD_STATUS_COMPLETE) {
				if (ret != CMD_STATUS_TIME_OUT)
					err("%s join_bss completed with %d",
					    dev->netdev->name, ret);
				else
					info("%s join_bss ssid %s timed out",
						     dev->netdev->name,
					     mac2str(dev->bss[dev->curr_bss].bssid));

				/* retry next BSS immediately */
				defer_kevent(dev,KEVENT_JOIN);
				goto end_join;
			}

			/* here we have joined the (I)BSS */
			if (dev->iw_mode == IW_MODE_ADHOC) {
				struct bss_info *bptr = dev->bss+dev->curr_bss;
				NEW_STATE(dev,CONNECTED);
				/* get ESSID, BSSID and channel for dev->curr_bss */
				dev->essid_size = bptr->ssid_len;
				memcpy(dev->essid, bptr->ssid, bptr->ssid_len);
				memcpy(dev->bssid, bptr->bssid, ETH_ALEN);
				dev->channel = bptr->channel;

				netif_start_queue(dev->netdev);
				/* just to be sure */
				del_timer_sync(&dev->mgmt_timer);
			} else {
				/* send auth req */
				NEW_STATE(dev,AUTHENTICATING);
				auth_req(dev,dev->curr_bss);
				mod_timer(&dev->mgmt_timer, jiffies+HZ);
			}
			goto end_join;
		} /* if ((dev->curr_bss=find_matching_bss(dev,0)) >= 0) */

		/* here we haven't found a matching (i)bss ... */
		if (dev->iw_mode == IW_MODE_ADHOC) {
			NEW_STATE(dev,STARTIBSS);
			defer_kevent(dev,KEVENT_STARTIBSS);
			goto end_join;
		}
		/* haven't found a matching BSS
		   in infra mode - use timer to try again in 10 seconds */
		NEW_STATE(dev,SCANNING);
		mod_timer(&dev->mgmt_timer, jiffies+RESCAN_TIME*HZ);
	} /* if (test_bit(KEVENT_JOIN, &dev->kevent_flags)) */
end_join:

	if (test_bit(KEVENT_SCAN, &dev->kevent_flags)) {
		clear_bit(KEVENT_SCAN, &dev->kevent_flags);
		assert(dev->istate == SCANNING);
		if ((ret=start_scan(dev)) < 0) {
			err("%s: start_scan failed with %d",
			    dev->netdev->name, ret);
			goto end_scan;
		}
		
		if ((ret=wait_completion(dev,CMD_SCAN)) !=
		    CMD_STATUS_COMPLETE) {
				err("%s start_scan completed with %d",
				    dev->netdev->name, ret);
				goto end_scan;
		}

		dump_bss_table(dev);
		NEW_STATE(dev,JOINING);
		assert(dev->curr_bss == -1); /* done in start_scan, 
						find_bss will start with index -1 + 1 */
		/* call join_bss immediately after
		   re-run of all other threads in kevent */
		defer_kevent(dev,KEVENT_JOIN);
	} /* if (test_bit(KEVENT_SCAN, &dev->kevent_flags)) */
end_scan:

	if (test_bit(KEVENT_SUBMIT_RX, &dev->kevent_flags)) {
		clear_bit(KEVENT_SUBMIT_RX, &dev->kevent_flags);
		submit_rx_urb(dev);
	}

	up(&dev->sem);

	return;
}

static
int essid_matched(struct at76c503 *dev, struct bss_info *ptr)
{
	static const u8 zeros[32];

	if (dev->iw_mode != IW_MODE_ADHOC) {
		if (ptr->ssid_len == 0)
			return 1; /* bss cloaked its ssid */
		if (ptr->ssid_len == 1 && ptr->ssid == '\0')
			return 1; /* Agere APs cloaking method */
		if (ptr->ssid_len == dev->essid_size &&
		    !memcmp(ptr->ssid, zeros, ptr->ssid_len))
			return 1; /* Belkin AP cloaking method */
	}

	/* common criteria for both modi */
	return dev->essid_size == 0 ||
		(dev->essid_size == ptr->ssid_len &&
		 !memcmp(dev->essid, ptr->ssid, ptr->ssid_len));
}

static inline
int mode_matched(struct at76c503 *dev, struct bss_info *ptr)
{
	if (dev->iw_mode == IW_MODE_ADHOC)
		return ptr->capa & IEEE802_11_CAPA_IBSS;
	else
		return ptr->capa & IEEE802_11_CAPA_ESS;
}

static
int rates_matched(struct at76c503 *dev, struct bss_info *ptr)
{
	int i;
	u8 *rate;

	for(i=0,rate=ptr->rates; i < ptr->rates_len; i++,rate++)
		if (*rate & 0x80) {
			/* this is a basic rate we have to support
			   (see IEEE802.11, ch. 7.3.2.2) */
			if (*rate != (0x80|hw_rates[0]) && *rate != (0x80|hw_rates[1]) &&
			    *rate != (0x80|hw_rates[2]) && *rate != (0x80|hw_rates[3])) {
				info("%s: bssid %s: basic rate %02x not supported",
				     dev->netdev->name, mac2str(ptr->bssid),*rate);
				return 0;
			}
		}
	return 1;
}

static inline
int wep_matched(struct at76c503 *dev, struct bss_info *ptr)
{
	if (!dev->wep_enabled && 
	    ptr->capa & IEEE802_11_CAPA_PRIVACY)
		/* we have disabled WEP, but the BSS signals privacy */
		return 0;
	/* otherwise if the BSS does not signal privacy it may well
	   except encrypted packets from us ... */
	return 1;
}

static void dump_bss_table(struct at76c503 *dev)
{
	struct bss_info *ptr;
	/* hex dump output buffer for debug */
	char hexssid[IW_ESSID_MAX_SIZE*2+1] __attribute__ ((unused));
	char hexrates[MAX_RATE_LEN*3+1] __attribute__ ((unused));
	int i;
	int nr=dev->bss_nr; /* copy it in case the rx callback adds
			       a new BSS */

	dbg("%s BSS table:", dev->netdev->name);
	for(i=0,ptr=dev->bss; i < nr; i++,ptr++) {
		dbg("%d.BSS %s channel %d ssid %s (%d:%s)"
		    " capa x%04x rates %s rssi %d link %d noise %d",
		    i, mac2str(ptr->bssid),
		    ptr->channel,
		    ptr->ssid, ptr->ssid_len,
		    hex2str(hexssid,ptr->ssid,ptr->ssid_len,'\0'),
		    le16_to_cpu(ptr->capa),
		    hex2str(hexrates, ptr->rates, 
			    ptr->rates_len, ' '),
		    ptr->rssi, ptr->link_qual, ptr->noise_level);
	}
}

/* try to find a matching bss in dev->bss, starting at index start.
   returns the first index of a matching bss in the array or
   -1 if none found */
static int find_matching_bss(struct at76c503 *dev, int start)
{
	struct bss_info *ptr;
	int i, ret;
	int nr = dev->bss_nr; /* copy it in case the rx callback adds
				 a new BSS */
	char hexssid[IW_ESSID_MAX_SIZE*2+1] __attribute__ ((unused));
	char ossid[IW_ESSID_MAX_SIZE+1];

	for(i=start, ptr = dev->bss + start; i < nr; i++, ptr++) {
		if (essid_matched(dev,ptr) &&
		    mode_matched(dev,ptr)  &&
		    wep_matched(dev,ptr)   &&
		    rates_matched(dev,ptr))
			break;
	}

	ret = i < nr ? i : -1;

	/* make dev->essid printable */
	assert(dev->essid_size <= IW_ESSID_MAX_SIZE);
	memcpy(ossid, dev->essid, dev->essid_size);
	ossid[dev->essid_size] = '\0';

	dbg("%s %s: try to match ssid %s (%s) mode %s wep %s, start at %d,"
	    " return %d",
	    dev->netdev->name, __FUNCTION__, ossid, 
	    hex2str(hexssid,dev->essid,dev->essid_size,'\0'),
	    dev->iw_mode == IW_MODE_ADHOC ? "adhoc" : "infra",
	    dev->wep_enabled ? "enabled" : "disabled", start, ret);

	return ret;
} /* find_matching_bss */


/* we got an association response */
static void rx_mgmt_assoc(struct at76c503 *dev,
			   struct at76c503_rx_buffer *buf)
{
	struct ieee802_11_mgmt *mgmt = (struct ieee802_11_mgmt *)buf->packet;
	struct ieee802_11_assoc_resp *resp = 
		(struct ieee802_11_assoc_resp *)mgmt->data;
	char orates[2*8+1] __attribute__((unused));

	dbg("%s: rx AssocResp bssid %s capa x%04x status x%04x "
	    "assoc_id x%04x rates %s",
	    dev->netdev->name, mac2str(mgmt->addr3),
	    le16_to_cpu(resp->capability), le16_to_cpu(resp->status),
	    le16_to_cpu(resp->assoc_id),
	    hex2str(orates, resp->data+2,
		    min((size_t)*(resp->data+1),(sizeof(orates)-1)/2), '\0'));

	if (dev->istate == ASSOCIATING) {
		if (resp->status == IEEE802_11_STATUS_SUCCESS) {
			struct bss_info *ptr = dev->bss+dev->curr_bss;
			dev->assoc_id = le16_to_cpu(resp->assoc_id);
			netif_carrier_on(dev->netdev);
			netif_wake_queue(dev->netdev); /* _start_queue ??? */
			NEW_STATE(dev,CONNECTED);
			dbg("%s: connected to BSSID %s",
			    dev->netdev->name, mac2str(dev->bss[dev->curr_bss].bssid));
			/* update iwconfig params */
			memcpy(dev->bssid, ptr->bssid, ETH_ALEN);
			memcpy(dev->essid, ptr->ssid, ptr->ssid_len);
			dev->essid_size = ptr->ssid_len;
			dev->channel = ptr->channel;
		} else {
			NEW_STATE(dev,JOINING);
			defer_kevent(dev,KEVENT_JOIN);
		}
		del_timer_sync(&dev->mgmt_timer);
	} else
		info("%s: AssocResp in state %d ignored",
		     dev->netdev->name, dev->istate);
} /* rx_mgmt_assoc */

static void rx_mgmt_reassoc(struct at76c503 *dev,
			   struct at76c503_rx_buffer *buf)
{
	struct ieee802_11_mgmt *mgmt = (struct ieee802_11_mgmt *)buf->packet;
	struct ieee802_11_assoc_resp *resp = 
		(struct ieee802_11_assoc_resp *)mgmt->data;
	char orates[2*8+1] __attribute__((unused));

	dbg("%s: rx ReAssocResp bssid %s capa x%04x status x%04x "
	    "assoc_id x%04x rates %s",
	    dev->netdev->name, mac2str(mgmt->addr3),
	    le16_to_cpu(resp->capability), le16_to_cpu(resp->status),
	    le16_to_cpu(resp->assoc_id),
	    hex2str(orates, resp->data+2,
		    min((size_t)*(resp->data+1),(sizeof(orates)-1)/2), '\0'));

	if (dev->istate == REASSOCIATING) {
		if (resp->status == IEEE802_11_STATUS_SUCCESS) {
			struct bss_info *bptr = dev->bss+dev->new_bss;
			dev->assoc_id = le16_to_cpu(resp->assoc_id);
			NEW_STATE(dev,CONNECTED);
			dev->curr_bss = dev->new_bss;
			/* get ESSID, BSSID and channel for dev->curr_bss */
			dev->essid_size = bptr->ssid_len;
			memcpy(dev->essid, bptr->ssid, bptr->ssid_len);
			memcpy(dev->bssid, bptr->bssid, ETH_ALEN);
			dev->channel = bptr->channel;
			dbg("%s: reassociated to BSSID %s",
			    dev->netdev->name, mac2str(dev->bssid));
			
			netif_carrier_on(dev->netdev);
			netif_wake_queue(dev->netdev);
		} else {
			del_timer_sync(&dev->mgmt_timer);
			NEW_STATE(dev,JOINING);
			defer_kevent(dev,KEVENT_JOIN);
		}
	} else
		info("%s: ReAssocResp in state %d ignored",
		     dev->netdev->name, dev->istate);
} /* rx_mgmt_reassoc */

static void rx_mgmt_disassoc(struct at76c503 *dev,
			   struct at76c503_rx_buffer *buf)
{
	struct ieee802_11_mgmt *mgmt = (struct ieee802_11_mgmt *)buf->packet;
	struct ieee802_11_disassoc_frame *resp = 
		(struct ieee802_11_disassoc_frame *)mgmt->data;
	char obuf[ETH_ALEN*3+1] __attribute__ ((unused));

	dbg("%s: rx DisAssoc bssid %s reason x%04x destination %s",
	    dev->netdev->name, mac2str(mgmt->addr3),
	    le16_to_cpu(resp->reason),
	    hex2str(obuf, mgmt->addr1, ETH_ALEN, ':'));

	if (!memcmp(mgmt->addr3,dev->bss[dev->curr_bss].bssid, ETH_ALEN) &&
		(!memcmp(dev->netdev->dev_addr, mgmt->addr1, ETH_ALEN) ||
			!memcmp(bc_addr, mgmt->addr1, ETH_ALEN))) {
		/* this is a DisAssoc from the BSS we are connected or
		   trying to connect to, directed to us or broadcasted */

		if (dev->istate == DISASSOCIATING ||
		    dev->istate == REASSOCIATING  ||
		    dev->istate == CONNECTED  ||
		    dev->istate == JOINING)
		{
			del_timer_sync(&dev->mgmt_timer);
			NEW_STATE(dev,JOINING);
			defer_kevent(dev,KEVENT_JOIN);
		} else
		/* ignore DisAssoc in states SCANNING, AUTH, ASSOC */
			info("%s: DisAssoc in state %d ignored",
			     dev->netdev->name, dev->istate);
	}
	/* ignore DisAssoc to other STA or from other BSSID */
} /* rx_mgmt_disassoc */

static void rx_mgmt_auth(struct at76c503 *dev,
			   struct at76c503_rx_buffer *buf)
{
	struct ieee802_11_mgmt *mgmt = (struct ieee802_11_mgmt *)buf->packet;
	struct ieee802_11_auth_frame *resp = 
		(struct ieee802_11_auth_frame *)mgmt->data;
	char obuf[ETH_ALEN*3+1] __attribute__ ((unused));

	dbg("%s: rx AuthFrame bssid %s alg %d seq_nr %d status %d " 
	    "destination %s",
	    dev->netdev->name, mac2str(mgmt->addr3),
	    le16_to_cpu(resp->algorithm),
	    le16_to_cpu(resp->seq_nr),
	    le16_to_cpu(resp->status),
	    hex2str(obuf, mgmt->addr1, ETH_ALEN, ':'));

	if (dev->istate != AUTHENTICATING)
		return;
	if (!memcmp(mgmt->addr3,dev->bss[dev->curr_bss].bssid, ETH_ALEN) &&
	    !memcmp(dev->netdev->dev_addr, mgmt->addr1, ETH_ALEN)) {
		/* this is a AuthFrame from the BSS we are connected or
		   trying to connect to, directed to us */
		if (resp->status != IEEE802_11_STATUS_SUCCESS) {
			del_timer_sync(&dev->mgmt_timer);
			/* try to join next bss */
			NEW_STATE(dev,JOINING);
			defer_kevent(dev,KEVENT_JOIN);
		} else {
			dev->retries = ASSOC_RETRIES;
			NEW_STATE(dev,ASSOCIATING);
			assoc_req(dev, dev->curr_bss);
			mod_timer(&dev->mgmt_timer,jiffies+HZ);
		}
	}
	/* else: ignore AuthFrames to other receipients */
} /* rx_mgmt_auth */

static void rx_mgmt_deauth(struct at76c503 *dev,
			   struct at76c503_rx_buffer *buf)
{
	struct ieee802_11_mgmt *mgmt = (struct ieee802_11_mgmt *)buf->packet;
	struct ieee802_11_deauth_frame *resp = 
		(struct ieee802_11_deauth_frame *)mgmt->data;
	char obuf[ETH_ALEN*3+1] __attribute__ ((unused));

	dbg("%s: rx DeAuth bssid %s reason x%04x destination %s",
	    dev->netdev->name, mac2str(mgmt->addr3),
	    le16_to_cpu(resp->reason),
	    hex2str(obuf, mgmt->addr1, ETH_ALEN, ':'));

	if (!memcmp(mgmt->addr3,dev->bss[dev->curr_bss].bssid, ETH_ALEN) &&
		(!memcmp(dev->netdev->dev_addr, mgmt->addr1, ETH_ALEN) ||
			!memcmp(bc_addr, mgmt->addr1, ETH_ALEN))) {
		/* this is a DeAuth from the BSS we are connected or
		   trying to connect to, directed to us or broadcasted */

		if (dev->istate == DISASSOCIATING ||
		    dev->istate == AUTHENTICATING ||
		    dev->istate == ASSOCIATING ||
		    dev->istate == REASSOCIATING  ||
		    dev->istate == CONNECTED  ||
		    dev->istate == JOINING)
		{
			NEW_STATE(dev,JOINING);
			defer_kevent(dev,KEVENT_JOIN);
			del_timer_sync(&dev->mgmt_timer);
		} else
		/* ignore DeAuth in states SCANNING */
		info("%s: DeAuth in state %d ignored",
		     dev->netdev->name, dev->istate);
	}
	/* ignore DeAuth to other STA or from other BSSID */
} /* rx_mgmt_deauth */

static void rx_mgmt_beacon(struct at76c503 *dev,
			   struct at76c503_rx_buffer *buf)
{
	struct ieee802_11_mgmt *mgmt = (struct ieee802_11_mgmt *)buf->packet;
	struct ieee802_11_beacon_data *bdata = 
		(struct ieee802_11_beacon_data *)mgmt->data;
	struct bss_info *bss_ptr;
	u8 *tlv_ptr;
	int i;

	if (dev->istate == CONNECTED) {
		/* in state CONNECTED we use the mgmt_timer to control
		   the beacon of the BSS */
		assert(dev->curr_bss >= 0);
		assert(dev->curr_bss < dev->bss_nr);
		bss_ptr = dev->bss+dev->curr_bss;
		if (!memcmp(bss_ptr->bssid,mgmt->addr3,ETH_ALEN)) {
			mod_timer(&dev->mgmt_timer, jiffies+BEACON_TIMEOUT*HZ);
			bss_ptr->rssi = buf->rssi;
			return;
		}
	}

	/* look if we have this BSS already in the list */
	for(i=0,bss_ptr=dev->bss; i < dev->bss_nr; i++,bss_ptr++) {
		if (!memcmp(bss_ptr->bssid,mgmt->addr3,ETH_ALEN))
			break;
	}

	if (i >= NR_BSS_INFO) {
		/* table is full */
		info("%s: scan bss table overflow",dev->netdev->name);
		return;
	}

	/* we either overwrite an existing entry or append a new one
	   bss_ptr points to the entry in both cases */

	/* capa is in little endian format (!) */
	bss_ptr->capa = bdata->capability_information;

	/* while beacon_interval is not (!) */
	bss_ptr->beacon_interval = le16_to_cpu(bdata->beacon_interval);

	bss_ptr->rssi = buf->rssi;
	bss_ptr->link_qual = buf->link_quality;
	bss_ptr->noise_level = buf->noise_level;

	memcpy(bss_ptr->mac,mgmt->addr2,ETH_ALEN); //just for info
	memcpy(bss_ptr->bssid,mgmt->addr3,ETH_ALEN);

	tlv_ptr = bdata->data;

	assert(*tlv_ptr == IE_ID_SSID);
	bss_ptr->ssid_len = min(IW_ESSID_MAX_SIZE,(int)*(tlv_ptr+1));
	memcpy(bss_ptr->ssid, tlv_ptr+2, bss_ptr->ssid_len);
	tlv_ptr += (1+1 + *(tlv_ptr+1));

	assert(*tlv_ptr == IE_ID_SUPPORTED_RATES);
	bss_ptr->rates_len = min(MAX_RATE_LEN,(int)*(tlv_ptr+1));
	memcpy(bss_ptr->rates, tlv_ptr+2, bss_ptr->rates_len);
	tlv_ptr += (1+1 + *(tlv_ptr+1));

	assert(*tlv_ptr == IE_ID_DS_PARAM_SET);
	bss_ptr->channel = *(tlv_ptr+2);

	if (i >= dev->bss_nr) {
		dev->bss_nr++;
	}
}

static void rx_mgmt(struct at76c503 *dev, struct at76c503_rx_buffer *buf)
{
	struct ieee802_11_mgmt *mgmt = ( struct ieee802_11_mgmt *)buf->packet;
	u16 subtype = mgmt->frame_ctl & IEEE802_11_FCTL_STYPE;

#if 0
	char obuf[128*2+1] __attribute__ ((unused));
	dbg("%s rx mgmt subtype x%x %s", dev->netdev->name, subtype,
	    hex2str(obuf, (u8 *)mgmt, 
		    min((sizeof(obuf)-1)/2,
			(size_t)le16_to_cpu(buf->wlength)), '\0'));
#endif

	switch (subtype) {
	case IEEE802_11_STYPE_BEACON:
	case IEEE802_11_STYPE_PROBE_RESP:
		rx_mgmt_beacon(dev,buf);
		break;

	case IEEE802_11_STYPE_ASSOC_RESP:
		rx_mgmt_assoc(dev,buf);
		break;

	case IEEE802_11_STYPE_REASSOC_RESP:
		rx_mgmt_reassoc(dev,buf);
		break;

	case IEEE802_11_STYPE_DISASSOC:
		rx_mgmt_disassoc(dev,buf);
		break;

	case IEEE802_11_STYPE_AUTH:
		rx_mgmt_auth(dev,buf);
		break;

	case IEEE802_11_STYPE_DEAUTH:
		rx_mgmt_deauth(dev,buf);
		break;

	default:
		info("%s: mgmt, but not beacon, subtype = %x",
		     dev->netdev->name, subtype);
	}

	return;
}

static void dbg_dumpbuf(const char *tag, const u8 *buf, int size)
{
	int i;

	if (!debug) return;

	for (i=0; i<size; i++) {
		if ((i % 8) == 0) {
			if (i) printk("\n");
			printk(KERN_DEBUG __FILE__ ": %s: ", tag);
		}
		printk("%02x ", buf[i]);
	}
	printk("\n");
}

/* Convert the 802.11 header on a packet into an ethernet-style header
 * (basically, pretend we're an ethernet card receiving ethernet packets)
 *
 * This routine returns with the skbuff pointing to the actual data (just past
 * the end of the newly-created ethernet header), and the CRC trimmed off.
 */
static void ieee80211_to_eth(struct sk_buff *skb, int iw_mode)
{
	struct ieee802_11_hdr *i802_11_hdr;
	struct ethhdr *eth_hdr;
	u8 *src_addr;
	u8 *dest_addr;
	unsigned short proto = 0;
	int build_ethhdr = 1;

	i802_11_hdr = (struct ieee802_11_hdr *)skb->data;
	skb_pull(skb, sizeof(struct ieee802_11_hdr));
	skb_trim(skb, skb->len - 4); /* Trim CRC */

	src_addr = iw_mode == IW_MODE_ADHOC ? i802_11_hdr->addr2
	       				    : i802_11_hdr->addr3;
	dest_addr = i802_11_hdr->addr1;

	eth_hdr = (struct ethhdr *)skb->data;
	if (!memcmp(eth_hdr->h_source, src_addr, ETH_ALEN) &&
	    !memcmp(eth_hdr->h_dest, dest_addr, ETH_ALEN)) {
		/* An ethernet frame is encapsulated within the data portion.
		 * Just use its header instead. */
		skb_pull(skb, sizeof(struct ethhdr));
		build_ethhdr = 0;
	} else if (!memcmp(skb->data, snapsig, sizeof(snapsig))) {
		/* SNAP frame. */
#ifdef COLLAPSE_RFC1042
		if (!memcmp(skb->data, rfc1042sig, sizeof(rfc1042sig))) {
			/* RFC1042 encapsulated packet.  Collapse it to a
			 * simple Ethernet-II or 802.3 frame */
			/* NOTE: prism2 doesn't collapse Appletalk frames (why?). */
			skb_pull(skb, sizeof(rfc1042sig)+2);
			proto = *(unsigned short *)(skb->data - 2);
		} else
#endif /* COLLAPSE_RFC1042 */
		proto = htons(skb->len);
	} else {
#if IEEE_STANDARD
		/* According to all standards, we should assume the data
		 * portion contains 802.2 LLC information, so we should give it
		 * an 802.3 header (which has the same implications) */
		proto = htons(skb->len);
#else /* IEEE_STANDARD */
		/* Unfortunately, it appears no actual 802.11 implementations
		 * follow any standards specs.  They all appear to put a
		 * 16-bit ethertype after the 802.11 header instead, so we take
		 * that value and make it into an Ethernet-II packet. */
		/* Note that this means we can never support non-SNAP 802.2
		 * frames (because we can't tell when we get one) */
		proto = *(unsigned short *)(skb->data);
		skb_pull(skb, 2);
#endif /* IEEE_STANDARD */
	}

	eth_hdr = (struct ethhdr *)(skb->data-sizeof(struct ethhdr));
	skb->mac.ethernet = eth_hdr;
	if (build_ethhdr) {
		/* This needs to be done in this order (eth_hdr->h_dest may
		 * overlap src_addr) */
		memcpy(eth_hdr->h_source, src_addr, ETH_ALEN);
		memcpy(eth_hdr->h_dest, dest_addr, ETH_ALEN);
		/* make an 802.3 header (proto = length) */
		eth_hdr->h_proto = proto;
	}

	/* TODO: check this max length */
	if (ntohs(eth_hdr->h_proto) >= 1536) {
		skb->protocol = eth_hdr->h_proto;
	} else if (*(unsigned short *)skb->data == 0xFFFF) {
		/* Magic hack for Novell IPX-in-802.3 packets */
		skb->protocol = htons(ETH_P_802_3);
	} else {
		/* Assume it's an 802.2 packet (it should be, and we have no
		 * good way to tell if it isn't) */
		skb->protocol = htons(ETH_P_802_2);
	}
}

/* Adjust the skb to trim the hardware header and CRC, and set up skb->mac,
 * skb->protocol, etc.
 */ 
static void ieee80211_fixup(struct sk_buff *skb, int iw_mode)
{
	struct ieee802_11_hdr *i802_11_hdr;
	struct ethhdr *eth_hdr;
	u8 *src_addr;
	u8 *dest_addr;
	unsigned short proto = 0;

	i802_11_hdr = (struct ieee802_11_hdr *)skb->data;
	skb_pull(skb, sizeof(struct ieee802_11_hdr));
	skb_trim(skb, skb->len - 4); /* Trim CRC */

	src_addr = iw_mode == IW_MODE_ADHOC ? i802_11_hdr->addr2
	       				    : i802_11_hdr->addr3;
	dest_addr = i802_11_hdr->addr1;

	skb->mac.raw = (unsigned char *)i802_11_hdr;

	eth_hdr = (struct ethhdr *)skb->data;
	if (!memcmp(eth_hdr->h_source, src_addr, ETH_ALEN) &&
	    !memcmp(eth_hdr->h_dest, dest_addr, ETH_ALEN)) {
		/* There's an ethernet header encapsulated within the data
		 * portion, count it as part of the hardware header */
		skb_pull(skb, sizeof(struct ethhdr));
		proto = eth_hdr->h_proto;
	} else if (!memcmp(skb->data, snapsig, sizeof(snapsig))) {
		/* SNAP frame */
#ifdef COLLAPSE_RFC1042
		if (!memcmp(skb->data, rfc1042sig, sizeof(rfc1042sig))) {
			/* RFC1042 encapsulated packet.  Treat the SNAP header
			 * as part of the HW header and note the protocol. */
			/* NOTE: prism2 doesn't collapse Appletalk frames (why?). */
			skb_pull(skb, sizeof(rfc1042sig) + 2);
			proto = *(unsigned short *)(skb->data - 2);
		} else
#endif /* COLLAPSE_RFC1042 */
		proto = htons(ETH_P_802_2);
	}

	/* TODO: check this max length */
	if (ntohs(proto) >= 1536) {
		skb->protocol = proto;
	} else {
#ifdef IEEE_STANDARD
		/* According to all standards, we should assume the data
		 * portion contains 802.2 LLC information */
		skb->protocol = htons(ETH_P_802_2);
#else /* IEEE_STANDARD */
		/* Unfortunately, it appears no actual 802.11 implementations
		 * follow any standards specs.  They all appear to put a
		 * 16-bit ethertype after the 802.11 header instead, so we'll
		 * use that (and consider it part of the hardware header). */
		/* Note that this means we can never support non-SNAP 802.2
		 * frames (because we can't tell when we get one) */
		skb->protocol = *(unsigned short *)(skb->data - 2);
		skb_pull(skb, 2);
#endif /* IEEE_STANDARD */
	}
}

static void rx_data(struct at76c503 *dev, struct at76c503_rx_buffer *buf)
{
	struct net_device *netdev = (struct net_device *)dev->netdev;
	struct net_device_stats *stats = &dev->stats;
	struct ieee802_11_hdr *i802_11_hdr = (struct ieee802_11_hdr *)buf->packet;
	struct sk_buff *skb = dev->rx_skb;
	int length;

	length = le16_to_cpu(buf->wlength);

	if (debug > 1) {
		dbg("%s received data packet:", netdev->name);
		dbg_dumpbuf(" rxhdr", skb->data, AT76C503_RX_HDRLEN);
		dbg_dumpbuf("packet", skb->data + AT76C503_RX_HDRLEN, length);
	}

	if (length < rx_copybreak && (skb = dev_alloc_skb(length)) != NULL) {
		memcpy(skb_put(skb, length),
			dev->rx_skb->data + AT76C503_RX_HDRLEN, length);
	} else {
		skb_pull(skb, AT76C503_RX_HDRLEN);
		skb_trim(skb, length);
		/* Use a new skb for the next receive */
		dev->rx_skb = NULL;
	}

	skb->dev = netdev;
	skb->ip_summed = CHECKSUM_NONE; /* TODO: should check CRC */

	if (i802_11_hdr->addr1[0] & 1) {
		if (!memcmp(i802_11_hdr->addr1, netdev->broadcast, ETH_ALEN))
			skb->pkt_type = PACKET_BROADCAST;
		else
			skb->pkt_type = PACKET_MULTICAST;
	} else if (memcmp(i802_11_hdr->addr1, netdev->dev_addr, ETH_ALEN)) {
		skb->pkt_type=PACKET_OTHERHOST;
	}

	if (netdev->type == ARPHRD_ETHER) {
		ieee80211_to_eth(skb, dev->iw_mode);
	} else {
		ieee80211_fixup(skb, dev->iw_mode);
	}

	netdev->last_rx = jiffies;
	netif_rx(skb);
	stats->rx_packets++;
	stats->rx_bytes += length;

	return;
}

static int submit_rx_urb(struct at76c503 *dev)
{
	int ret, size;
	struct sk_buff *skb = dev->rx_skb;

	if (skb == NULL) {
		skb = dev_alloc_skb(sizeof(struct at76c503_rx_buffer));
		if (skb == NULL) {
			err("%s: unable to allocate rx skbuff.", dev->netdev->name);
			ret = -ENOMEM;
			goto exit;
		}
		dev->rx_skb = skb;
	} else {
		skb_push(skb, skb_headroom(skb));
		skb_trim(skb, 0);
	}

	size = skb_tailroom(skb);
	usb_fill_bulk_urb(dev->read_urb, dev->udev,
		usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
		skb_put(skb, size), size,
		at76c503_read_bulk_callback, dev);
	ret = usb_submit_urb(dev->read_urb);
	if (ret < 0) {
		err("%s: rx, usb_submit_urb failed: %d", dev->netdev->name, ret);
	}

exit:
	if (ret < 0) {
		/* If we can't submit the URB, the adapter becomes completely
		 * useless, so try again later */
		/* TODO: should we limit the number of retries? */
		defer_kevent(dev, KEVENT_SUBMIT_RX);
	}
	return ret;
}


/* we are doing a lot of things here in an interrupt. Need
   a bh handler (Watching TV with a TV card is probably
   a good test: if you see flickers, we are doing too much.
   Currently I do see flickers... even with our tasklet :-( )
   Maybe because the bttv driver and usb-uhci use the same interrupt
*/
/* Or maybe because our BH handler is preempting bttv's BH handler.. BHs don't
 * solve everything.. (alex) */
static void at76c503_read_bulk_callback (struct urb *urb)
{
	struct at76c503 *dev = (struct at76c503 *)urb->context;

	dev->rx_urb = urb;
	tasklet_schedule(&dev->tasklet);

	return;
}

static void rx_tasklet(unsigned long param)
{
	struct at76c503 *dev = (struct at76c503 *)param;
	struct urb *urb = dev->rx_urb;
	struct net_device *netdev = (struct net_device *)dev->netdev;
	struct at76c503_rx_buffer *buf = (struct at76c503_rx_buffer *)dev->rx_skb->data;
	struct ieee802_11_hdr *i802_11_hdr = (struct ieee802_11_hdr *)buf->packet;
	u8 frame_type;
//	int flags;

	if(!urb) return; // paranoid

	if(urb->status != 0){
		if ((urb->status != -ENOENT) && 
		    (urb->status != -ECONNRESET)) {
			dbg("%s %s: - nonzero read bulk status received: %d",
			    __FUNCTION__, netdev->name, urb->status);
			goto next_urb;
		}
		return;
	}

	//info("%s: rxrate %d\n",netdev->name,buf->rx_rate);

	/* there is a new bssid around, accept it: */
	if(buf->newbss && dev->iw_mode == IW_MODE_ADHOC){
		dbg("%s: rx newbss", netdev->name);
		defer_kevent(dev, KEVENT_NEW_BSS);
	}

	/* no need to care about endianness of constants - is taken care
	   of in ieee802_11.h */
	frame_type = i802_11_hdr->frame_ctl & IEEE802_11_FCTL_FTYPE;

	if(frame_type == IEEE802_11_FTYPE_DATA){
//		info("rx: it's a data frame");
		rx_data(dev, buf);
	}else if(frame_type == IEEE802_11_FTYPE_MGMT){
//		info("rx: it's a mgmt frame");
		rx_mgmt(dev, buf);
	}else if(frame_type == IEEE802_11_FTYPE_CTL)
		dbg("%s: received ctrl frame: %2x", dev->netdev->name,
		    i802_11_hdr->frame_ctl);
	else
		dbg("%s: it's a frame from mars: %2x", dev->netdev->name,
		    i802_11_hdr->frame_ctl);

 next_urb:
	submit_rx_urb(dev);
	return;
}

static void at76c503_write_bulk_callback (struct urb *urb)
{
	struct at76c503 *dev = (struct at76c503 *)urb->context;
	struct net_device_stats *stats = &dev->stats;
	unsigned long flags;
	struct at76c503_tx_buffer *mgmt_buf;
	int ret;

	if(urb->status != 0){
		if((urb->status != -ENOENT) && 
		   (urb->status != -ECONNRESET)) {
			dbg("%s - nonzero write bulk status received: %d",
			    __FUNCTION__, urb->status);
		}else
			return; /* urb has been unlinked */
		stats->tx_errors++;
	}else
		stats->tx_packets++;

	spin_lock_irqsave(&dev->mgmt_spinlock, flags);
	mgmt_buf = dev->next_mgmt_bulk;
	dev->next_mgmt_bulk = NULL;
	spin_unlock_irqrestore(&dev->mgmt_spinlock, flags);

	if (mgmt_buf) {
		memcpy(dev->bulk_out_buffer, mgmt_buf,
		       le16_to_cpu(mgmt_buf->wlength) +
		       le16_to_cpu(mgmt_buf->padding) +
		       offsetof(struct at76c503_tx_buffer,packet));
		FILL_BULK_URB(dev->write_urb, dev->udev,
			      usb_sndbulkpipe(dev->udev, 
					      dev->bulk_out_endpointAddr),
			      dev->bulk_out_buffer,
			      le16_to_cpu(mgmt_buf->wlength) + 
			      le16_to_cpu(mgmt_buf->padding) +
			      offsetof(struct at76c503_tx_buffer,packet),
			      at76c503_write_bulk_callback, dev);
		ret = usb_submit_urb(dev->write_urb);
		if (ret) {
			err("%s: %s error in tx submit urb: %d",
			    dev->netdev->name, __FUNCTION__, ret);
		}
		kfree(mgmt_buf);
	} else
		netif_wake_queue(dev->netdev);

}

static int
at76c503_tx(struct sk_buff *skb, struct net_device *netdev)
{
	struct at76c503 *dev = (struct at76c503 *)(netdev->priv);
	struct net_device_stats *stats = &dev->stats;
	int ret;
	int len = skb->len < ETH_ZLEN ? ETH_ZLEN : skb->len;
	int wlen = len + 18;
	int submit_len;
	struct at76c503_tx_buffer *tx_buffer =
		(struct at76c503_tx_buffer *)dev->bulk_out_buffer;
	struct ieee802_11_hdr *i802_11_hdr =
		(struct ieee802_11_hdr *)&(tx_buffer->packet);

	/* we can get rid of memcpy, if we set netdev->hard_header_len
	   to 8 + sizeof(struct ieee802_11_hdr), because then we have
	   enough space */
	//  dbg("skb->data - skb->head = %d", skb->data - skb->head);

	/* 18 = sizeof(ieee802_11_hdr) - 2 * ETH_ALEN */
	/* ssap and dsap stay in the data */
	memcpy(&(tx_buffer->packet[18]), skb->data, len);
  
	/* make wireless header */
	/* no need to care about endianness of constants - is taken care
	   of in ieee802_11.h */
	i802_11_hdr->frame_ctl = IEEE802_11_FTYPE_DATA;

	/* jal TODO: if the destination has sent us unencrypted packets
	   and we did not exclude them, do not encrypt the answers.
	   -> have a table of peers */
	if (dev->wep_enabled)
		i802_11_hdr->frame_ctl |= IEEE802_11_FCTL_WEP;

	if(dev->iw_mode == IW_MODE_ADHOC){
		memcpy(i802_11_hdr->addr1, skb->data, ETH_ALEN); /* destination */
		memcpy(i802_11_hdr->addr2, netdev->dev_addr, ETH_ALEN); /* source */
		memcpy(i802_11_hdr->addr3, dev->bssid, ETH_ALEN);
	}else if(dev->iw_mode == IW_MODE_INFRA){
		i802_11_hdr->frame_ctl |= IEEE802_11_FCTL_TODS;
		memcpy(i802_11_hdr->addr1, dev->bssid, ETH_ALEN);
		memcpy(i802_11_hdr->addr2, netdev->dev_addr, ETH_ALEN); /* source */
		memcpy(i802_11_hdr->addr3, skb->data, ETH_ALEN); /* destination */
	}
	memset(i802_11_hdr->addr4, 0, ETH_ALEN);

	i802_11_hdr->duration_id = 0;
	i802_11_hdr->seq_ctl = 0;

	/* setup 'atmel' header */
	tx_buffer->wlength = cpu_to_le16(wlen);
	tx_buffer->tx_rate = dev->txrate; 
        /* for broadcast destination addresses, the firmware 0.100.x 
	   seems to choose the highest rate set with CMD_STARTUP in
	   basic_rate_set replacing this value */
	

	//info("txrate %d\n", dev->txrate);

#if 0 /* does not seem to be necessary */
	{
		/* strange things, as done in atmelwlandriver
		   - I do not really understand this (oku) */
		int tmp = wlen%64;
		tx_buffer->padding = tmp < 50 ? 50 - tmp : 114 - tmp;
	}
#endif
	memset(tx_buffer->reserved, 0, 4);

	tx_buffer->padding = 0;
	submit_len = wlen + 8 + tx_buffer->padding;

	/* send stuff */
	netif_stop_queue(netdev);
	netdev->trans_start = jiffies;

	FILL_BULK_URB(dev->write_urb, dev->udev,
		      usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
		      tx_buffer, submit_len,
		      at76c503_write_bulk_callback, dev);
	ret = usb_submit_urb(dev->write_urb);
	if(ret){
		stats->tx_errors++;
		err("%s: error in tx submit urb: %d", netdev->name, ret);
		goto err;
	}

	stats->tx_bytes += len;

	dev_kfree_skb(skb);
	return 0;

 err:
	return ret;
}

static
void at76c503_tx_timeout(struct net_device *netdev)
{
	struct at76c503 *dev = (struct at76c503 *)(netdev->priv);

	if (!dev)
		return;
	warn("%s: tx timeout.", netdev->name);
	dev->write_urb->transfer_flags |= USB_ASYNC_UNLINK;
	usb_unlink_urb(dev->write_urb);
	dev->stats.tx_errors++;
}

static
int at76c503_open(struct net_device *netdev)
{
	struct at76c503 *dev = (struct at76c503 *)(netdev->priv);
	int ret = 0;

	if(down_interruptible(&dev->sem))
	   return -EINTR;

	{
		struct at76c503_card_config *ccfg = &dev->card_config;

		memset(ccfg, 0, sizeof(struct at76c503_card_config));
		ccfg->exclude_unencrypted = dev->wep_excl_unencr;
		ccfg->promiscuous_mode = 0;
		ccfg->promiscuous_mode = 1;
		ccfg->short_retry_limit = 8;

		if (dev->wep_enabled &&
		    dev->wep_keys_len[dev->wep_key_id] > WEP_SMALL_KEY_LEN)
			ccfg->encryption_type = 2;
		else
			ccfg->encryption_type = 1;


                ccfg->rts_threshold = dev->rts_threshold;
                ccfg->fragmentation_threshold = dev->frag_threshold;

		memcpy(ccfg->basic_rate_set, hw_rates, 4);
		/* jal: really needed, we do a set_mib for autorate later ??? */
		ccfg->auto_rate_fallback = (dev->txrate == 4 ? 1 : 0);
		ccfg->channel = dev->channel;
		ccfg->privacy_invoked = dev->wep_enabled;
		memcpy(ccfg->current_ssid, dev->essid, IW_ESSID_MAX_SIZE);
		ccfg->ssid_len = dev->essid_size;

                ccfg->wep_default_key_id = dev->wep_key_id;
                memcpy(ccfg->wep_default_key_value, dev->wep_keys, 4 * WEP_KEY_SIZE);

		ccfg->short_preamble = dev->preamble_type;
		ccfg->beacon_period = 100;
	}

	ret = set_card_command(dev->udev, CMD_STARTUP, (unsigned char *)&dev->card_config,
			       sizeof(struct at76c503_card_config));
	if(ret < 0){
		err("%s: set_card_command failed: %d", netdev->name, ret);
		goto err;
	}

	wait_completion(dev, CMD_STARTUP);

	set_radio(dev, 1);
	wait_completion(dev, CMD_RADIO);

	set_preamble(dev, dev->preamble_type);
	set_frag(dev, dev->frag_threshold);
	set_rts(dev, dev->rts_threshold);
	set_autorate_fallback(dev, dev->txrate == 4 ? 1 : 0);

	ret = submit_rx_urb(dev);
	if(ret < 0){
		err("%s: open: submit_rx_urb failed: %d", netdev->name, ret);
		goto err;
	}

  	NEW_STATE(dev,SCANNING);
  	defer_kevent(dev,KEVENT_SCAN);
  	netif_carrier_off(dev->netdev); /* disable running netdev watchdog */
  	netif_stop_queue(dev->netdev); /* stop tx data packets */
   
	dbg("at76c503_open end");
 err:
	up(&dev->sem);
	return ret < 0 ? ret : 0;
}

static
int at76c503_stop(struct net_device *netdev)
{
	struct at76c503 *dev = (struct at76c503 *)(netdev->priv);
	unsigned long flags;

	down(&dev->sem);

	netif_stop_queue(netdev);

	set_radio(dev, 0);

	usb_unlink_urb(dev->read_urb);
	usb_unlink_urb(dev->write_urb);
	usb_unlink_urb(dev->ctrl_urb);

	del_timer_sync(&dev->mgmt_timer);

	spin_lock_irqsave(&dev->mgmt_spinlock,flags);
	if (dev->next_mgmt_bulk) {
		kfree(dev->next_mgmt_bulk);
		dev->next_mgmt_bulk = NULL;
	}
	spin_unlock_irqrestore(&dev->mgmt_spinlock,flags);

	up(&dev->sem);

	return 0;
}

static
struct net_device_stats *at76c503_get_stats(struct net_device *netdev)
{
	struct at76c503 *dev = (struct at76c503 *)netdev->priv;
        
	return &dev->stats;
}

static
struct iw_statistics *at76c503_get_wireless_stats(struct net_device *netdev)
{
	struct at76c503 *dev = (struct at76c503 *)netdev->priv;
        
	return &dev->wstats;
}

static
void at76c503_set_multicast(struct net_device *netdev)
{
	struct at76c503 *dev = (struct at76c503 *)netdev->priv;
	int promisc;

	promisc = ((netdev->flags & IFF_PROMISC) != 0);
	if(promisc != dev->promisc){
		/* grmbl. This gets called in interrupt. */
		dev->promisc = promisc;
		defer_kevent(dev, KEVENT_SET_PROMISC);
	}
}

static
int at76c503_set_mac_address(struct net_device *netdev, void *addr)
{
	struct at76c503 *dev = (struct at76c503 *)netdev->priv;
	struct sockaddr *mac = addr;
	int ret;

	if (down_interruptible(&dev->sem))
                return -EINTR;

	ret = set_mac_address(dev, mac->sa_data);
	if(ret >= 0){
		memcpy(netdev->dev_addr, mac->sa_data, ETH_ALEN);
	}

	up(&dev->sem);

	return ret;
}

static
int at76c503_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	struct at76c503 *dev = netdev->priv;
	struct iwreq *wrq = (struct iwreq *)rq;
	int ret = 0;
	//  int changed = 0;
  
	if (! netif_device_present(netdev))
		return -ENODEV;

	if (down_interruptible(&dev->sem))
                return -EINTR;

	switch (cmd) {
	case SIOCGIWNAME:
		dbg("%s: SIOCGIWNAME", netdev->name);
		strcpy(wrq->u.name, "IEEE 802.11-DS");
		break;
                
	case SIOCGIWAP:
		dbg("%s: SIOCGIWAP", netdev->name);
		wrq->u.ap_addr.sa_family = ARPHRD_ETHER;

		memcpy(wrq->u.ap_addr.sa_data, dev->bssid, ETH_ALEN);

		break;

	case SIOCSIWRTS:
		dbg("%s: SIOCSIWRTS", netdev->name);
		{
			int rthr = wrq->u.rts.value;
#if 0 
			printk(KERN_DEBUG "%s: ioctl SIOCSIWRTS, value %d, disabled %d\n",
			       dev->name, wrq->u.rts.value, wrq->u.rts.disabled);
#endif
			if(wrq->u.rts.disabled)
				rthr = MAX_RTS_THRESHOLD;
			if((rthr < 0) || (rthr > MAX_RTS_THRESHOLD)) {
				ret = -EINVAL;
				goto error;
			}
			dev->rts_threshold = rthr;
		}
		break;

	// Get the current RTS threshold
	case SIOCGIWRTS:
		dbg("%s: SIOCGIWRTS", netdev->name);
#if 0
		printk(KERN_DEBUG "%s: ioctl SIOCGIWRTS\n", dev->name);
#endif
		wrq->u.rts.value = dev->rts_threshold;
		wrq->u.rts.disabled = (wrq->u.rts.value == MAX_RTS_THRESHOLD);
		wrq->u.rts.fixed = 1;
		break;

		// Set the desired fragmentation threshold

       case SIOCSIWFRAG:
		dbg("%s: SIOCSIWFRAG", netdev->name);
		{
			int fthr = wrq->u.frag.value;
#if 0
			printk(KERN_DEBUG "%s: ioctl SIOCSIWFRAG, value %d, disabled %d\n", 
			       dev->name, wrq->u.frag.value, wrq->u.frag.disabled);
#endif
			if(wrq->u.frag.disabled)
				fthr = MAX_FRAG_THRESHOLD;
			if((fthr < MIN_FRAG_THRESHOLD) || (fthr > MAX_FRAG_THRESHOLD)){
				ret = -EINVAL;
				goto error;
			}else{
				dev->frag_threshold = fthr & ~0x1; // get an even value
			}
		}
		break;

       // Get the current fragmentation threshold
       case SIOCGIWFRAG:
		dbg("%s: SIOCGIWFRAG", netdev->name);

		wrq->u.frag.value = dev->frag_threshold;
		wrq->u.frag.disabled = (wrq->u.frag.value >= MAX_FRAG_THRESHOLD);
		wrq->u.frag.fixed = 1;
		break;

	case SIOCGIWFREQ:
		dbg("%s: SIOCGIWFREQ", netdev->name);
		wrq->u.freq.m = dev->channel;
		wrq->u.freq.e = 0;
		wrq->u.freq.i = 0;
		break;

	case SIOCSIWFREQ:
		dbg("%s: SIOCGIWFREQ", netdev->name);
		/* copied from orinoco.c */
		{
			struct iw_freq *frq = &wrq->u.freq;
			int chan = -1;

			/* We can only use this in Ad-Hoc demo mode to set the operating
			 * frequency, or in IBSS mode to set the frequency where the IBSS
			 * will be created - Jean II */
			/* dunno if this also applies for this device (oku) */
			if(dev->iw_mode != IW_MODE_ADHOC){
				ret = -EOPNOTSUPP;
				goto error;
			}

			if((frq->e == 0) && (frq->m <= 1000)){
				/* Setting by channel number */
				chan = frq->m;
			}else{
				/* Setting by frequency - search the table */
				int mult = 1;
				int i;
	
				for(i = 0; i < (6 - frq->e); i++)
					mult *= 10;
	
				for(i = 0; i < NUM_CHANNELS; i++)
					if(frq->m == (channel_frequency[i] * mult))
						chan = i+1;
			}
      
			/* TODO: we should also check if the channel is
			   supported by the device (oku) */
			if ((chan < 1) || (chan > NUM_CHANNELS)){
				ret = -EINVAL;
				goto error;
			}

			dev->channel = chan;
		}
		break;

	case SIOCGIWMODE:
		dbg("%s: SIOCGIWMODE", netdev->name);
		wrq->u.mode = dev->iw_mode;
		break;

	case SIOCSIWMODE:
		dbg("%s: SIOCSIWMODE %d", netdev->name, wrq->u.mode);
		dev->iw_mode = wrq->u.mode;
		break;

	case SIOCGIWESSID:
		dbg("%s: SIOCGIWESSID", netdev->name);
		{
			char essidbuf[IW_ESSID_MAX_SIZE];
			struct iw_point *erq = &wrq->u.essid;

			memcpy(essidbuf, dev->essid, sizeof(essidbuf));
			erq->flags = 1;
			erq->length = strlen(essidbuf);

			if(erq->pointer){
				if(copy_to_user(erq->pointer, essidbuf, erq->length)){
					ret = -EFAULT;
					goto error;
				}
			}

		}
		break;

	case SIOCSIWESSID:
		dbg("%s: SIOCSIWESSID", netdev->name);
		{
			char essidbuf[IW_ESSID_MAX_SIZE];
			struct iw_point *erq = &wrq->u.essid;

			memset(&essidbuf, 0, sizeof(essidbuf));

			if (erq->flags) {
				if (erq->length > IW_ESSID_MAX_SIZE){
					ret = -E2BIG;
					goto error;
				}
	
				if (copy_from_user(essidbuf, erq->pointer, erq->length)){
					ret = -EFAULT;
					goto error;
				}
	
				/* iwconfig gives len including 0 byte -
				   3 hours debugging... grrrr (oku) */
				dev->essid_size = erq->length - 1; 
				memcpy(dev->essid, essidbuf, IW_ESSID_MAX_SIZE);
			}
		}
		break;

	case SIOCGIWRATE:
		wrq->u.bitrate.value = dev->txrate == 0 ? 1000000 :
			dev->txrate == 1 ? 2000000 : dev->txrate == 2 ? 5500000 : 
			dev->txrate == 3 ? 11000000 : 11000000;
		wrq->u.bitrate.fixed = (dev->txrate != 4);
		wrq->u.bitrate.disabled = 0;
		break;

	case SIOCSIWRATE:
		dbg("%s: SIOCSIWRATE %d", netdev->name, wrq->u.bitrate.value);
		switch (wrq->u.bitrate.value){
		case -1: dev->txrate = 4; break; /* auto rate */ 
		case 1000000: dev->txrate = 0; break;
		case 2000000: dev->txrate = 1; break;
		case 5500000: dev->txrate = 2; break;
		case 11000000: dev->txrate = 3; break;
		default:
			ret = -EINVAL;
		}
		break;

        case SIOCSIWENCODE:
		dbg("%s: SIOCSIWENCODE", netdev->name);
#if 1
		printk(KERN_DEBUG "%s: ioctl SIOCSIWENCODE enc.flags %08x "
		       "pointer %p len %d\n",
		       dev->netdev->name, wrq->u.encoding.flags, 
		       wrq->u.encoding.pointer, wrq->u.encoding.length);
		printk(KERN_DEBUG "%s: old wepstate: enabled %d key_id %d "
		       "excl_unencr %d\n",
		       dev->netdev->name, dev->wep_enabled, dev->wep_key_id,
		       dev->wep_excl_unencr);
#endif
		{
			int index = (wrq->u.encoding.flags & IW_ENCODE_INDEX) - 1;
                        /* take the old default key if index is invalid */
			if((index < 0) || (index >= NR_WEP_KEYS))
				index = dev->wep_key_id;
			if(wrq->u.encoding.pointer){
				int len = wrq->u.encoding.length;

				if(len > WEP_LARGE_KEY_LEN){
					len = WEP_LARGE_KEY_LEN;
				}

				memset(dev->wep_keys[index], 0, WEP_KEY_SIZE);
				if(copy_from_user(dev->wep_keys[index], 
						   wrq->u.encoding.pointer, len)) {
					dev->wep_keys_len[index] = 0;
					ret = -EFAULT;
				}else{
					dev->wep_keys_len[index] =
						len <= WEP_SMALL_KEY_LEN ?
						WEP_SMALL_KEY_LEN : WEP_LARGE_KEY_LEN;
#if 0
					printk(KERN_DEBUG "%s: new key index %d, len %d: ",
					       dev->netdev->name, index, dev->wep_keys_len[index]);
#endif
					dev->wep_enabled = 1;
				}
			}
      
			dev->wep_key_id = index;
			dev->wep_enabled = ((wrq->u.encoding.flags & IW_ENCODE_DISABLED) == 0);
			if (wrq->u.encoding.flags & IW_ENCODE_RESTRICTED)
				dev->wep_excl_unencr = 1;
			if (wrq->u.encoding.flags & IW_ENCODE_OPEN)
				dev->wep_excl_unencr = 0;
#if 1
			printk(KERN_DEBUG "%s: new wepstate: enabled %d key_id %d key_len %d "
			       "excl_unencr %d\n",
			       dev->netdev->name, dev->wep_enabled, dev->wep_key_id,
			       dev->wep_keys_len[dev->wep_key_id],
			       dev->wep_excl_unencr);
#endif
		}
		break;

		// Get the WEP keys and mode
	case SIOCGIWENCODE:
		dbg("%s: SIOCGIWENCODE", netdev->name);
		{
			int index = (wrq->u.encoding.flags & IW_ENCODE_INDEX) - 1;
			if ((index < 0) || (index >= NR_WEP_KEYS))
				index = dev->wep_key_id;

			wrq->u.encoding.flags = 
				(dev->wep_excl_unencr) ? IW_ENCODE_RESTRICTED : IW_ENCODE_OPEN;
			if(!dev->wep_enabled)
				wrq->u.encoding.flags |= IW_ENCODE_DISABLED;
			if(wrq->u.encoding.pointer){
				wrq->u.encoding.length = dev->wep_keys_len[index];
				if (copy_to_user(wrq->u.encoding.pointer, 
						 dev->wep_keys[index],
						 dev->wep_keys_len[index]))
					ret = -EFAULT;
				wrq->u.encoding.flags |= (index + 1);
			}
		}
		break;

	default:
		dbg("%s: ioctl not supported (0x%x)", netdev->name, cmd);
		ret = -EOPNOTSUPP;
	}

 error:
	up(&dev->sem);
	return ret;
}

void at76c503_delete_device(struct at76c503 *dev)
{
	if(dev){
		unregister_netdevice(dev->netdev);

		if(dev->bulk_out_buffer != NULL)
			kfree(dev->bulk_out_buffer);
		if(dev->ctrl_buffer != NULL)
			kfree(dev->ctrl_buffer);

		if(dev->write_urb != NULL)
			usb_free_urb(dev->write_urb);
		if(dev->read_urb != NULL)
			usb_free_urb(dev->read_urb);
		if(dev->rx_skb != NULL)
			kfree_skb(dev->rx_skb);
		if(dev->ctrl_buffer != NULL)
			usb_free_urb(dev->ctrl_urb);
    
		kfree (dev->netdev); /* dev is in net_dev */ 
	}
}

static int at76c503_alloc_urbs(struct at76c503 *dev)
{
	struct usb_interface *interface = dev->interface;
	struct usb_interface_descriptor *iface_desc = &interface->altsetting[0];
	struct usb_endpoint_descriptor *endpoint;
	struct usb_device *udev = dev->udev;
	int i, buffer_size;

	for(i = 0; i < iface_desc->bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i];

		if ((endpoint->bEndpointAddress & 0x80) &&
		    ((endpoint->bmAttributes & 3) == 0x02)) {
			/* we found a bulk in endpoint */

			dev->read_urb = usb_alloc_urb(0);
			if (!dev->read_urb) {
				err("No free urbs available");
				return -1;
			}
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
		}
		
		if (((endpoint->bEndpointAddress & 0x80) == 0x00) &&
		    ((endpoint->bmAttributes & 3) == 0x02)) {
			/* we found a bulk out endpoint */
			dev->write_urb = usb_alloc_urb(0);
			if (!dev->write_urb) {
				err("no free urbs available");
				return -1;
			}
			buffer_size = sizeof(struct at76c503_tx_buffer);
			dev->bulk_out_size = buffer_size;
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_out_buffer = kmalloc (buffer_size, GFP_KERNEL);
			if (!dev->bulk_out_buffer) {
				err("couldn't allocate bulk_out_buffer");
				return -1;
			}
			FILL_BULK_URB(dev->write_urb, udev, 
				      usb_sndbulkpipe(udev, 
						      endpoint->bEndpointAddress),
				      dev->bulk_out_buffer, buffer_size,
				      at76c503_write_bulk_callback, dev);
		}
	}

	dev->ctrl_urb = usb_alloc_urb(0);
	if (!dev->ctrl_urb) {
		err("no free urbs available");
		return -1;
	}
	dev->ctrl_buffer = kmalloc(1024, GFP_KERNEL);
	if (!dev->ctrl_buffer) {
		err("couldn't allocate ctrl_buffer");
		return -1;
	}

	return 0;
}

struct at76c503 *at76c503_new_device(struct usb_device *udev, int board_type, const char *netdev_name)
{
	struct net_device *netdev;
	struct at76c503 *dev = NULL;
	struct usb_interface *interface;
	int ret;

	/* allocate memory for our device state and intialize it */
	netdev = alloc_etherdev(sizeof(struct at76c503));
	if (netdev == NULL) {
		err("out of memory");
		goto error;
	}

	dev = (struct at76c503 *)netdev->priv;
	dev->udev = udev;
	dev->netdev = netdev;

	init_MUTEX (&dev->sem);
	INIT_TQUEUE (&dev->kevent, kevent, dev);

	init_timer(&dev->mgmt_timer);
	dev->mgmt_timer.data = (unsigned long)dev;
	dev->mgmt_timer.function = mgmt_timeout;
	dev->mgmt_spinlock = SPIN_LOCK_UNLOCKED;
	dev->next_mgmt_bulk = NULL;
	dev->istate = INIT;
	dev->curr_bss = dev->new_bss = -1;

	dev->tasklet.func = rx_tasklet;
	dev->tasklet.data = (unsigned long)dev;

	dev->board_type = board_type;

	/* set up the endpoint information */
	/* check out the endpoints */
	interface = &udev->actconfig->interface[0];
	dev->interface = interface;

	if(at76c503_alloc_urbs(dev) < 0)
		goto error;

	/* get firmware version */
	ret = get_mib(dev->udev, MIB_FW_VERSION, (u8*)&dev->fw_version, sizeof(dev->fw_version));
	if((ret < 0) || ((dev->fw_version.major == 0) && 
			 (dev->fw_version.minor == 0) && 
			 (dev->fw_version.patch == 0) && 
			 (dev->fw_version.build == 0))){
		err("getting firmware failed with %d, or version is 0", ret);
		err("this probably means that the ext. fw was not loaded correctly");
		goto error;
	}
	info("firmware version %d.%d.%d #%d",
	     dev->fw_version.major, dev->fw_version.minor,
	     dev->fw_version.patch, dev->fw_version.build);

	/* MAC address */
	ret = get_hw_config(dev);
	if(ret < 0){
		err("could not get MAC address");
		goto error;
	}
	memcpy(netdev->dev_addr, dev->mac_addr, ETH_ALEN);
	info("using MAC %s", mac2str(netdev->dev_addr));

	set_mac_address(dev, netdev->dev_addr); /* may have been changed, write back original */

	/* initializing */
	dev->channel = DEF_CHANNEL;
	dev->iw_mode = IW_MODE_ADHOC;
	memset(dev->essid, 0, IW_ESSID_MAX_SIZE);
	memcpy(dev->essid, DEF_ESSID, DEF_ESSID_LEN);
	dev->essid_size = DEF_ESSID_LEN;
	dev->rts_threshold = DEF_RTS_THRESHOLD;
	dev->frag_threshold = DEF_FRAG_THRESHOLD;
	dev->txrate = TX_RATE_AUTO;
	dev->preamble_type = PREAMBLE_TYPE_LONG;

	netdev->flags &= ~IFF_MULTICAST; /* not yet or never */
	netdev->open = at76c503_open;
	netdev->stop = at76c503_stop;
	netdev->get_stats = at76c503_get_stats;
	netdev->get_wireless_stats = at76c503_get_wireless_stats;
	netdev->hard_start_xmit = at76c503_tx;
	netdev->tx_timeout = at76c503_tx_timeout;
	netdev->do_ioctl = at76c503_ioctl;
	netdev->set_multicast_list = at76c503_set_multicast;
	netdev->set_mac_address = at76c503_set_mac_address;
	strcpy(netdev->name, netdev_name);
	//  netdev->hard_header_len = 8 + sizeof(struct ieee802_11_hdr);
	/*
//    netdev->hard_header = at76c503_header;
*/
	return dev;

 error:
	at76c503_delete_device(dev);
	return NULL;

}

struct at76c503 *at76c503_do_probe(struct module *mod, struct usb_device *udev, u8 *extfw, int extfw_size, int board_type, const char *netdev_name)
{
	int ret;
	struct at76c503 *dev;

	usb_inc_dev_use(udev);

	if ((ret = usb_get_configuration(udev)) != 0) {
		err("get configuration failed: %d", ret);
		goto error;
	}

	if ((ret = usb_set_configuration(udev, 1)) != 0) {
		err("set configuration to 1 failed: %d", ret);
		goto error;
	}

	if (extfw && extfw_size) {
		ret = at76c503_download_external_fw(udev, extfw, extfw_size);
		if (ret < 0) {
			err("Downloading external firmware failed: %d", ret);
			goto error;
		}
	}

	dev = at76c503_new_device(udev, board_type, netdev_name);
	if (!dev) {
		dbg("at76c503_new_device returned NULL");
		goto error;
	}

	dev->netdev->owner = mod;
	ret = register_netdev(dev->netdev);
	if (ret) {
		err("Unable to register netdevice %s (status %d)!",
		    dev->netdev->name, ret);
		at76c503_delete_device(dev);
		goto error;
	}

	return dev;

error:
	usb_dec_dev_use(udev);
	return NULL;
}

/**
 * 	at76c503_usbdfu_post
 *
 * 	Called by usbdfu driver after the firmware has been downloaded, before
 * 	the final reset.
 * 	(this is called in a usb probe (khubd) context)
 */

int at76c503_usbdfu_post(struct usb_device *udev)
{
	int result;

	dbg("Sending remap command...");
	result = at76c503_remap(udev);
	if (result < 0) {
		err("Remap command failed (%d)", result);
		return result;
	}
	return 0;
}

/**
 *	at76c503_init
 */
static int __init at76c503_init(void)
{
	info(DRIVER_DESC " " DRIVER_VERSION);
	return 0;
}

/**
 *	at76c503_exit
 */
static void __exit at76c503_exit(void)
{
	info(DRIVER_DESC " " DRIVER_VERSION " exit");
}

module_init (at76c503_init);
module_exit (at76c503_exit);

EXPORT_SYMBOL(at76c503_do_probe);
EXPORT_SYMBOL(at76c503_download_external_fw);
EXPORT_SYMBOL(at76c503_new_device);
EXPORT_SYMBOL(at76c503_delete_device);
EXPORT_SYMBOL(at76c503_usbdfu_post);
EXPORT_SYMBOL(at76c503_remap);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
