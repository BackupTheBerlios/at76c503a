/* -*- linux-c -*- */
/*
 * USB Device Firmware Upgrade (DFU) handler
 *
 * Copyright (c) 2003 Oliver Kurth <oku@masqmail.cx>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation; either version 2 of
 *	the License, or (at your option) any later version.
 *
 * 2003_01_19 0.1:
 * - initial release
 *
 * TODO:
 * (someday)
 * - make a way for drivers to feed firmware data at download time (instead of
 *   providing it all at once during register)
 * - procfs support for userland firmware downloaders
 * - Firmware upload (device-to-host) support
 */

#include <linux/config.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include "usbdfu.h"

#ifdef CONFIG_USB_DEBUG
static int debug = 1;
#else
static int debug;
#endif

/* Use our own dbg macro */
#undef dbg
#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg); } while (0)

#ifdef DEBUG_SEM
  #define dfu_down(sem) do { dbg("sem %s down", #sem); down(sem); } while (0)
  #define dfu_up(sem) do { dbg("sem %s up", #sem); up(sem); } while (0)
#else
  #define dfu_down(sem) down(sem)
  #define dfu_up(sem) up(sem)
#endif

/* Version Information */
#define DRIVER_AUTHOR \
"Oliver Kurth <oku@masqmail.cx>, Joerg Albert <joerg.albert@gmx.de>, Alex <alex@foogod.com>"
#define DRIVER_DESC "USB Device Fimware Upgrade (DFU) handler"

/* Module paramaters */
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Debug enabled or not");

/* USB class/subclass for DFU devices/interfaces */

#define DFU_USB_CLASS    0xfe
#define DFU_USB_SUBCLASS 0x01

/* DFU states */

#define STATE_IDLE  			0x00
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

struct dfu_status {
	unsigned char bStatus;
	unsigned char bwPollTimeout[3];
	unsigned char bState;
	unsigned char iString;
} __attribute__ ((packed));

struct usbdfu_infolist {
	struct list_head list;
	struct usbdfu_info *info;
};

/* driver independent download context */
struct dfu_ctx {
	struct usb_device *udev;
	u8 dfu_state;
	struct dfu_status dfu_status;
	u8 *buf;
};

#define KEVENT_FLAG_SCHEDRESET 1
#define KEVENT_FLAG_RESET 2

/* Structure to hold all of our device specific stuff */
struct usbdfu {
	struct usb_device *	udev;			/* save off the usb device pointer */

	struct timer_list timer;

	struct tq_struct	kevent;
	u32 kevent_flags;
		
	struct semaphore	sem;			/* locks this structure */

	struct usbdfu_info *info;
	u8 op_mode;
};

LIST_HEAD(usbdfu_infolist_head);
struct semaphore usbdfu_lock;

/* local function prototypes */
static void * usbdfu_probe(struct usb_device *dev,
			   unsigned int ifnum, const struct usb_device_id *id);
static void usbdfu_disconnect(struct usb_device *dev, void *ptr);

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver usbdfu_driver = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,20)
	owner:      THIS_MODULE,
#endif
	name:       "usbdfu",
	probe:      usbdfu_probe,
	disconnect: usbdfu_disconnect,
	id_table:   NULL,
};

/**
 *	usbdfu_debug_data
 */
static inline void usbdfu_debug_data (const char *function, int size, const unsigned char *data)
{
	int i;

	if (!debug)
		return;
	
	printk (KERN_DEBUG __FILE__": %s - length = %d, data = ", 
		function, size);
	for (i = 0; i < size; ++i) {
		printk ("%.2x ", data[i]);
	}
	printk ("\n");
}


#define USB_SUCCESS(a) (a >= 0)

#define DFU_PACKETSIZE 1024

#define INTERFACE_VENDOR_REQUEST_OUT 0x41
#define INTERFACE_VENDOR_REQUEST_IN  0xc1

static
int dfu_detach(struct usb_device *udev)
{
	int result;

	dbg("dfu_detach");

	result = usb_control_msg(udev, usb_sndctrlpipe(udev,0),
				 DFU_DETACH,
				 USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				 1000,	/* Value */
				 0,	/* Index */
				 NULL,	/* Buffer */
				 0,	/* Size */
				 HZ);

	return result;
}

static
int dfu_download_block(struct dfu_ctx *ctx, u8 *buffer,
		       int bytes, int block)
{
	int result;
	u8 *tmpbuf = ctx->buf;
	struct usb_device *udev = ctx->udev;

	dbg("dfu_download_block(): buffer=%p, bytes=%d, block=%d", buffer, bytes, block);

	if(tmpbuf == NULL)
		return -ENOMEM;

	memcpy(tmpbuf, buffer, bytes);

	result = usb_control_msg(udev, usb_sndctrlpipe(udev,0),
				 DFU_DNLOAD,
				 USB_TYPE_CLASS | USB_DIR_OUT | USB_RECIP_INTERFACE,
				 block,	/* Value */
				 0,	/* Index */
				 tmpbuf,	/* Buffer */
				 bytes,	/* Size */
				 HZ);
	return result;
}

static
int dfu_get_status(struct dfu_ctx *ctx, struct dfu_status *status)
{
	int result;
	struct usb_device *udev = ctx->udev;

//	dbg("dfu_get_status()");

	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
				 DFU_GETSTATUS,
				 USB_TYPE_CLASS | USB_DIR_IN | USB_RECIP_INTERFACE,
				 0,	/* Value */
				 0,	/* Index */
				 status,	/* Buffer */
				 sizeof(struct dfu_status),	/* Size */
				 HZ);

	return result;
}

static
u8 dfu_get_state(struct usb_device *udev, u8 *state)
{
	int result;

//	dbg("dfu_get_state()");

	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
				 DFU_GETSTATE,	/* Request */
				 USB_TYPE_CLASS | USB_DIR_IN | USB_RECIP_INTERFACE,
				 0,	/* Value */
				 0,	/* Index */
				 state,	/* Buffer */
				 1,	/* Size */
				 HZ);

	return result;
}

static inline
u32 __get_timeout(struct dfu_status *s)
{
	unsigned long ret = 0;

	ret  = (unsigned long) (s->bwPollTimeout[2] << 16);
	ret |= (unsigned long) (s->bwPollTimeout[1] << 8);
	ret |= (unsigned long) (s->bwPollTimeout[0]);

	return ret;
}

static
struct dfu_ctx *dfu_alloc_ctx(struct usb_device *udev)
{
	struct dfu_ctx *ctx;

	ctx = kmalloc(sizeof(struct dfu_ctx) + DFU_PACKETSIZE, GFP_KERNEL|GFP_DMA);
	if(ctx){
		ctx->udev = udev;
		ctx->buf = (u8 *)&(ctx[1]);
	}
	return ctx;
}

int do_dfu_download(struct usb_device *udev, unsigned char *dfu_buffer,
		 unsigned int dfu_len)
{
	struct dfu_ctx *ctx;
	struct dfu_status *dfu_stat_buf;
	int status = 0;
	int need_dfu_state = 1;
	int is_done = 0;
	u8 dfu_state = 0;
	u32 dfu_timeout = 0;
	int dfu_block_bytes = 0, dfu_bytes_left = dfu_len, dfu_buffer_offset = 0;
	int dfu_block_cnt = 0;

	if (dfu_len == 0) {
		err("FW Buffer length invalid!");
		return -EINVAL;
	}

	ctx = dfu_alloc_ctx(udev);
	if(ctx == NULL)
		return -ENOMEM;

	dfu_stat_buf = &ctx->dfu_status;

	do {
		if (need_dfu_state) {
			status = dfu_get_state(ctx->udev, &ctx->dfu_state);
			if (!USB_SUCCESS(status)) {
				err("DFU: Failed to get DFU state: %d", status);
				goto exit;
			}
			dfu_state = ctx->dfu_state;
			need_dfu_state = 0;
		}

		switch (dfu_state) {
		case STATE_DFU_DOWNLOAD_SYNC:
			dbg("STATE_DFU_DOWNLOAD_SYNC");
			if (USB_SUCCESS
			    (status = dfu_get_status(ctx, dfu_stat_buf))) {
				dfu_state = dfu_stat_buf->bState;
				dfu_timeout = __get_timeout(dfu_stat_buf);
				need_dfu_state = 0;
			}else
				err("dfu_get_status failed with %d", status);
			break;

		case STATE_DFU_DOWNLOAD_BUSY:
			dbg("STATE_DFU_DOWNLOAD_BUSY");
			need_dfu_state = 1;

			if (dfu_timeout >= 0){
				dbg("DFU: Resetting device");
				set_current_state( TASK_INTERRUPTIBLE );
				schedule_timeout(1+dfu_timeout*HZ/1000);
			}else
				dbg("DFU: In progress");

			break;

		case STATE_DFU_DOWNLOAD_IDLE:
			dbg("DOWNLOAD...");
			/* fall through */
		case STATE_DFU_IDLE:
			dbg("DFU IDLE");

			if (dfu_bytes_left <= DFU_PACKETSIZE)
				dfu_block_bytes = dfu_bytes_left;
			else
				dfu_block_bytes = DFU_PACKETSIZE;

			dfu_bytes_left -= dfu_block_bytes;
			status = dfu_download_block(ctx,
					      dfu_buffer +
					      dfu_buffer_offset,
					      dfu_block_bytes,
					      dfu_block_cnt);
			dfu_buffer_offset += dfu_block_bytes;
			dfu_block_cnt++;

			if (!USB_SUCCESS(status))
				err("dfu_download_block failed with %d", status);
			need_dfu_state = 1;
			break;

		case STATE_DFU_MANIFEST_SYNC:
			dbg("STATE_DFU_MANIFEST_SYNC");

			status = dfu_get_status(ctx, dfu_stat_buf);

			if (USB_SUCCESS(status)) {
				dfu_state = dfu_stat_buf->bState;
				dfu_timeout = __get_timeout(dfu_stat_buf);
				need_dfu_state = 0;

				if (dfu_timeout >= 0){
					dbg("DFU: Waiting for manifest phase");

					set_current_state( TASK_INTERRUPTIBLE );
					schedule_timeout((dfu_timeout*HZ+999)/1000);
				}else
					dbg("DFU: In progress");
			}
			break;

		case STATE_DFU_MANIFEST:
			dbg("STATE_DFU_MANIFEST");
			is_done = 1;
			break;

		case STATE_DFU_MANIFEST_WAIT_RESET:
			dbg("STATE_DFU_MANIFEST_WAIT_RESET");
//			usb_reset_device(udev);
			is_done = 1;
			break;

		case STATE_DFU_UPLOAD_IDLE:
			dbg("STATE_DFU_UPLOAD_IDLE");
			break;

		case STATE_DFU_ERROR:
			dbg("STATE_DFU_ERROR");
//			usb_reset_device(udev);
			status = -EPIPE;
			break;

		default:
			dbg("DFU UNKNOWN STATE (%d)", dfu_state);
			status = -EINVAL;
			break;
		}
	} while (!is_done && USB_SUCCESS(status));

 exit:
	kfree(ctx);
	if (status < 0)
		return status;
	else
		return 0;
}

static
int usbdfu_download(struct usbdfu *dev, u8 *fw_buf, u32 fw_len)
{
	int ret = 0;

	if (dev->info->pre_download_hook) {
		ret = dev->info->pre_download_hook(dev->udev);
	}

	if (ret)
		return ret;

	info("Downloading firmware for USB device %d...", dev->udev->devnum);

	ret = do_dfu_download(dev->udev, fw_buf, fw_len);

	if (ret)
		return ret;

	if (dev->info->post_download_hook) {
		ret = dev->info->post_download_hook(dev->udev);
	}

	return ret;
}

static inline void usbdfu_delete (struct usbdfu *dev)
{
	kfree (dev);
}

/* shamelessly copied from usbnet.c (oku) */
static void defer_kevent (struct usbdfu *dev, int flag)
{
	set_bit (flag, &dev->kevent_flags);
	if (!schedule_task (&dev->kevent))
		err ("kevent %d may have been dropped", flag);
	else
		dbg ("kevent %d scheduled", flag);
}

static void kevent_timer(unsigned long data)
{
	struct usbdfu *dev = (struct usbdfu *)data;

	defer_kevent(dev, KEVENT_FLAG_RESET);

/* jal: this hangs SMP systems. no need to stop the timer, as
   it is non-periodic */
//	del_timer_sync(&dev->timer);
}

/* TODO: how do we make sure the device hasn't been
   plugged out in the meantime? */
/* We don't really need to (trying to reset a disconnected device shouldn't
 * cause a problem), we just need to make sure that disconnect hasn't freed the
 * dev structure already.  We do this by not freeing dev as long as
 * "kevent_flags" has something set (indicating a kevent is pending)  --alex */

static void
kevent(void *data)
{
	struct usbdfu *dev = data;
	struct usb_device *udev;
	struct usbdfu_info *info;
	struct usb_interface *interface;

	dbg("kevent entered");

	/* some paranoid checks: */
	if(!dev){
		err("kevent: no dev!");
		return;
	}

	dfu_down(&dev->sem);

	info = dev->info;
	if(!info){
		err("kevent: no dev->info!");
		goto exit;
	}
	udev = dev->udev;
	if(!udev){
		err("kevent: no device!");
		goto exit;
	}

	if (test_bit(KEVENT_FLAG_SCHEDRESET, &dev->kevent_flags)) {
		clear_bit(KEVENT_FLAG_SCHEDRESET, &dev->kevent_flags);
		defer_kevent (dev, KEVENT_FLAG_RESET);
	} else if (test_bit(KEVENT_FLAG_RESET, &dev->kevent_flags)) {
		clear_bit(KEVENT_FLAG_RESET, &dev->kevent_flags);

		/* releasing interface, so it can be claimed by our
		   fellow driver */
		interface = &udev->actconfig->interface[0];
		usb_driver_release_interface(&usbdfu_driver, interface);

		/* Once we release the interface, the USB system won't call
		 * usbdfu_disconnect for us, so we need to do that ourselves.
		 * Note: we cannot use dev after this point. */
		dfu_up(&dev->sem);
		usbdfu_disconnect(udev, dev);

		dbg("resetting device");
		usb_reset_device(udev);

		dbg("scanning unclaimed devices");
		usb_scan_devices();

		return;
	}

 exit:
	dfu_up(&dev->sem);
	return;
}

int usbdfu_register(struct usbdfu_info *info)
{
	struct usbdfu_infolist *infolist = kmalloc(sizeof(struct usbdfu_infolist), GFP_KERNEL);

	if(!infolist)
		return -ENOMEM;

	infolist->info = info;

	dfu_down(&usbdfu_lock);
	list_add_tail(&infolist->list, &usbdfu_infolist_head);
	dfu_up(&usbdfu_lock); /* before scan, because that calls probe() */

	dbg("registered new driver %s", info->name);

	/* if the device is not yet plugged in, we are settled. If it already
	   is (and it's already in DFU state), we have to scan for unclaimed
	   devices. This will call our probe function again. */
	usb_scan_devices();
	
	return 0;
}

void usbdfu_deregister(struct usbdfu_info *info)
{
	struct list_head *tmp;
	struct usbdfu_infolist *infolist = NULL;

	dbg("deregistering driver %s", info->name);
	dfu_down(&usbdfu_lock);

        for(tmp = usbdfu_infolist_head.next;
	      tmp != &usbdfu_infolist_head;
	      tmp = tmp->next) {

                infolist = list_entry(tmp, struct usbdfu_infolist,
				 list);

		if(infolist->info == info)
			break;
        }
	if(tmp != &usbdfu_infolist_head){
		list_del(tmp);
		kfree(infolist);
	} else {
		err("unregistering %s: driver was not previously registered!",
		    info->name);
	}
	dfu_up(&usbdfu_lock);
}

int usbdfu_in_use(struct usb_device *udev, unsigned int ifnum)
{
	int result;
       	u8 state;
	struct usb_interface *interface;
	struct usb_interface_descriptor *idesc;

	if (ifnum != 0) {
		/* DFU-mode devices only have one interface */
		return 0;
	}

	/* Check to see whether the interface's class is a DFU device.
	 * We need to check this first to make sure the DFU_GETSTATE command
	 * isn't misinterpreted as something else. */
	interface = &udev->actconfig->interface[ifnum];
	idesc = &interface->altsetting[interface->act_altsetting];
	if ((idesc->bInterfaceClass != DFU_USB_CLASS) ||
	    (idesc->bInterfaceSubClass != DFU_USB_SUBCLASS)) {
		dbg("interface class is not DFU");
		return 0;
	}

	result = dfu_get_state(udev, &state);
	if (result < 0) {
		return result;
	} else if (result != 1) {
		/* This should be an error.  The device reported this interface
		 * as a DFU-class interface, but it's not responding correctly
		 * to DFU-class commands on this interface.  However, there
		 * appear to be some broken devices out there where this is
		 * normal behavior in some cases (at76c503 immediately after
		 * fw-load-reset), so just continue on (and hope we didn't
		 * screw anything up with that DFU command).. */
		dbg("DFU state query returned %d-byte response",
		    result);
		return 0;
	}

	switch (state) {
	  case STATE_IDLE:
	  case STATE_DETACH:
		/* Device is in an application mode, it's up to other drivers
		 * to deal with it */
		dbg("DFU state=App (%d)", state);
		return 0;
	  case STATE_DFU_IDLE:
	  case STATE_DFU_DOWNLOAD_SYNC:
	  case STATE_DFU_DOWNLOAD_BUSY:
	  case STATE_DFU_DOWNLOAD_IDLE:
	  case STATE_DFU_MANIFEST_SYNC:
	  case STATE_DFU_MANIFEST:
	  case STATE_DFU_MANIFEST_WAIT_RESET:
	  case STATE_DFU_UPLOAD_IDLE:
	  case STATE_DFU_ERROR:
		/* This is what we're looking for.  We're in the middle
		 * of dealing with this device */
		dbg("DFU state=DFU (%d)", state);
		return 1;
	  default:
		/* We got something that shouldn't be a valid response to a DFU
		 * state query.  Again, this sometimes happens on broken
		 * devices (at76c503 immediately after fw-load-reset) which
		 * report DFU class but aren't really DFU-capable. */
		dbg("DFU state query returned bizarre response (%d)", state);
		return 0;
	}
}

static struct usbdfu_info *find_info(struct usb_device *udev)
{
	struct usb_interface *interface;
	struct list_head *tmp;
	struct usbdfu_infolist *infolist;

	dbg("searching for driver");

	interface = &udev->actconfig->interface[0];

        for(tmp = usbdfu_infolist_head.next;
	      tmp != &usbdfu_infolist_head;
	      tmp = tmp->next) {
		infolist = list_entry(tmp, struct usbdfu_infolist,
				 list);
		if(usb_match_id(udev, interface, infolist->info->id_table))
			return infolist->info;
	}

	return NULL;
}

int usbdfu_initiate_download(struct usb_device *udev)
{
	int result;

	if (!find_info(udev)) {
		return -ENOENT;
	}

	result = dfu_detach(udev);
	if (!result) {
		dbg("dfu_detach failed (%d)", result);
		return result;
	}

	usb_reset_device(udev);
	usb_scan_devices();

	return 0;
}

static void * usbdfu_probe(struct usb_device *udev, unsigned int ifnum, const struct usb_device_id *id)
{
	struct usbdfu *dev = NULL;
	struct usbdfu_info *info = NULL;
	int ret;

	dbg("usbdfu_probe entered");

	if (ifnum != 0) {
		dbg("more than one interface, cannot be DFU mode");
		return NULL;
	}

	dfu_down(&usbdfu_lock);

	info = find_info(udev);
	if (!info)
		goto exit; /* not for us */

	dbg("device is registered (%s)", info->name);

	if (usbdfu_in_use(udev, ifnum) != 1) {
		dbg("device not in DFU-idle mode");
		goto exit;
	}
	dbg("device is in DFU mode");

	/* allocate memory for our device state and intialize it */
	dev = kmalloc (sizeof(struct usbdfu), GFP_KERNEL);
	if (dev == NULL) {
		err ("out of memory");
		goto exit;
	}
	memset (dev, 0, sizeof (*dev));

	init_MUTEX (&dev->sem);

	dfu_down(&dev->sem);

	INIT_TQUEUE (&dev->kevent, kevent, dev);
	dev->udev = udev;
	dev->info = info;

	dbg("going for download");
	/* here our main action takes place: */
	ret = usbdfu_download(dev, info->fw_buf, info->fw_buf_len);
	if(ret < 0){
		err("Firmware download failed for USB device %d", udev->devnum);
		goto error;
	}

	init_timer(&dev->timer);

	if(info->reset_delay){
		dev->timer.data = (long) dev;
		dev->timer.function = kevent_timer;
		
		mod_timer(&dev->timer, jiffies + info->reset_delay);
	}else{
		defer_kevent (dev, KEVENT_FLAG_SCHEDRESET);
	}

	dfu_up(&dev->sem);
	goto exit;
	
error:
	dfu_up(&dev->sem);
	usbdfu_delete (dev);
	dev = NULL;

exit:
	dfu_up(&usbdfu_lock);

	dbg("usbdfu_probe() exiting");
	return dev;
}


static void usbdfu_disconnect(struct usb_device *udev, void *ptr)
{
	struct usbdfu *dev = (struct usbdfu *)ptr;
	int kevent_pending;

	dbg("usbdfu_disconnect called");

	while (1) {
		dfu_down(&dev->sem);
		kevent_pending = dev->kevent_flags;
		dfu_up(&dev->sem);
		if (!kevent_pending) break;
		dbg("usbdfu_disconnect: waiting for kevent to complete (%d pending)...", kevent_pending);
		schedule();
	}

	del_timer_sync(&dev->timer);

	usbdfu_delete(dev);

	dbg("USB DFU now disconnected");
}

static int __init usbdfu_init(void)
{
	int result;

	info(DRIVER_DESC " " DRIVER_VERSION);

	init_MUTEX(&usbdfu_lock);

	/* register this driver with the USB subsystem */
	result = usb_register(&usbdfu_driver);
	if (result < 0) {
		err("usb_register failed for the "__FILE__" driver. Error number %d",
		    result);
		return -1;
	}

	return 0;
}

/**
 *	usbdfu_exit
 */
static void __exit usbdfu_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&usbdfu_driver);
}

module_init (usbdfu_init);
module_exit (usbdfu_exit);

EXPORT_SYMBOL(usbdfu_register);
EXPORT_SYMBOL(usbdfu_deregister);
EXPORT_SYMBOL(usbdfu_in_use);
EXPORT_SYMBOL(usbdfu_initiate_download);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

