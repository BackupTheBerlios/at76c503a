/* -*- linux-c -*- */
/*
 * USB Device Firmware Uploader
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
 * - make it more generic, so that it works with devices other than
 *   at76c503
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


/* Version Information */
#define DRIVER_AUTHOR "Oliver Kurth oku@masqmail.cx"
#define DRIVER_DESC "USB Device Fimware Uploader (DFU)"

/* Module paramaters */
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Debug enabled or not");

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

struct usbdfu_driver {
	struct list_head list;
	struct usb_driver *driver;
	u8 *fw_buf;
	u32 fw_buf_len;
};

/* driver independent download context */
struct dfu_ctx {
	struct usb_device *udev;
	u8 dfu_state;
	struct dfu_status dfu_status;
	u8 *buf;
};

#define KEVENT_FLAG_AGAIN 1
#define KEVENT_FLAG_REMAP 2
#define KEVENT_FLAG_RESET 3

/* Structure to hold all of our device specific stuff */
struct usbdfu {
	struct usb_device *	udev;			/* save off the usb device pointer */
	struct usb_interface *	interface;		/* the interface for this device */

	struct timer_list timer_reset;

	struct tq_struct	kevent;
	u32 kevent_flags;
		
	struct semaphore	sem;			/* locks this structure */

	struct usbdfu_driver *drv;
	u8 op_mode;
};

LIST_HEAD(usbdfu_driver_list);
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

#if 0
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
#endif

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
u8 dfu_get_state(struct dfu_ctx *ctx, u8 *state)
{
	int result;
	struct usb_device *udev = ctx->udev;

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

int dfu_download(struct usb_device *udev, unsigned char *dfu_buffer,
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
			status = dfu_get_state(ctx, &ctx->dfu_state);
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
					dbg("DFU: Resetting device");

					set_current_state( TASK_INTERRUPTIBLE );
					schedule_timeout(1+dfu_timeout*HZ/1000);
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
	return status;
}

static inline
int remap(struct usbdfu *dev)
{
	struct usb_device *udev = dev->udev;
	return usb_control_msg(udev, usb_sndctrlpipe(udev,0),
			 0x0a, INTERFACE_VENDOR_REQUEST_OUT,
			 0, 0,
			 NULL, 0, HZ);
}

static inline
int get_op_mode(struct usbdfu *dev, u8 *op_mode)
{
	struct usb_device *udev = dev->udev;
	int ret;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev,0),
			      0x33, INTERFACE_VENDOR_REQUEST_IN,
			      0x01, 0,
			      op_mode, 1, HZ);
	return ret;
}

static
int usbdfu_download(struct usbdfu *dev, u8 *fw_buf, u32 fw_len)
{
	int ret;

	info("downloading firmware");

	ret = dfu_download(dev->udev, fw_buf, fw_len);

	if(ret >= 0){
		info("remap");
		ret = remap(dev);
		if(ret < 0)
			err("remap failed: %d", ret);
	}

	return ret;
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

static void kevent_timer_reset(unsigned long data)
{
	struct usbdfu *dev = (struct usbdfu *)data;

	defer_kevent(dev, KEVENT_FLAG_RESET);

	del_timer_sync(&dev->timer_reset);
}

/* TODO: how do we make sure the device hasn't been
   plugged out in the meantime? */

static void
kevent(void *data)
{
	struct usbdfu *dev = data;
	struct usb_driver *driver;
	struct usb_device *udev;
	struct usbdfu_driver *drv;
	struct usb_interface *interface;

	dbg("kevent entered");

	/* some paranoid checks: */
	if(!dev){
		err("kevent: no dev!");
		return;
	}

	down(&dev->sem);

	if(!dev->drv){
		err("kevent: no dev->drv!");
		goto error;
	}
	driver = dev->drv->driver;
	if(!driver){
		err("kevent: no driver!");
		goto error;
	}
	udev = dev->udev;
	if(!udev){
		err("kevent: no device!");
		goto error;
	}
	drv = dev->drv;
	interface = dev->interface;

	up(&dev->sem);

	/* releasing interface, so it can be claimed by our
	   fellow driver */
	usb_driver_release_interface(&usbdfu_driver, dev->interface);

	info("resetting device");
	usb_reset_device(udev);

	info("scanning unclaimed devices");
	usb_scan_devices();

	/* we cannot use dev after reset - disconnect may have been
	   been called, it's gone */

	return;
 error:
	up(&dev->sem);
	return;
}

int usbdfu_register(struct usb_driver *driver, u8 *fw_buf, int fw_buf_len)
{
	struct usbdfu_driver *drv = kmalloc(sizeof(struct usbdfu_driver), GFP_KERNEL);

	if(!drv)
		return -ENOMEM;

	drv->fw_buf = fw_buf;
	drv->fw_buf_len = fw_buf_len;
	drv->driver = driver;

	down(&usbdfu_lock);
	list_add_tail(&drv->list, &usbdfu_driver_list);
	up(&usbdfu_lock); /* before scan, because that calls probe() */

	/* if the device is not yet plugged in, we are settled. If it already
	   is, we have to scan for unclaimed devices. This will call our
	   probe function again. */
	/* I hope this works: */
	usb_scan_devices();
	
	return 0;
}

void usbdfu_deregister(struct usb_driver *driver)
{
	struct list_head *tmp;
	struct usbdfu_driver *drv;

	down(&usbdfu_lock);

        for(tmp = usbdfu_driver_list.next;
	      tmp != &usbdfu_driver_list;
	      tmp = tmp->next) {

                drv = list_entry(tmp, struct usbdfu_driver,
				 list);

		if(drv->driver == driver)
			break;
        }
	if(tmp != &usbdfu_driver_list){
		list_del(&drv->list);
		kfree(drv);
	}
	up(&usbdfu_lock);
}

static inline void usbdfu_delete (struct usbdfu *dev)
{
	kfree (dev);
}

static void * usbdfu_probe(struct usb_device *udev, unsigned int ifnum, const struct usb_device_id *id)
{
	struct usbdfu *dev = NULL;
	struct usb_interface *interface;
	struct list_head *tmp;
	struct usbdfu_driver *drv = NULL;
	int ret;

	dbg("usbdfu_probe entered");

	if(ifnum != 0){
		info("cannot handle multiple interfaces");
		return NULL;
	}

	interface = &udev->actconfig->interface[ifnum];

	down(&usbdfu_lock);

	dbg("searching driver");

        for(tmp = usbdfu_driver_list.next;
	      tmp != &usbdfu_driver_list;
	      tmp = tmp->next) {
		drv = list_entry(tmp, struct usbdfu_driver,
				 list);
		if(usb_match_id(udev, interface, drv->driver->id_table))
			break;
	}

	if(tmp == &usbdfu_driver_list)
		goto exit; /* not for us */

	dbg("driver %s found", drv->driver->name);

	/* allocate memory for our device state and intialize it */
	dev = kmalloc (sizeof(struct usbdfu), GFP_KERNEL);
	if (dev == NULL) {
		err ("out of memory");
		goto exit;
	}
	memset (dev, 0, sizeof (*dev));

	init_MUTEX (&dev->sem);

	down(&dev->sem);

	INIT_TQUEUE (&dev->kevent, kevent, dev);
	dev->udev = udev;
	dev->interface = interface;
	dev->drv = drv;

	ret = get_op_mode(dev, &dev->op_mode);
	if(ret < 0){
		/* this is not a problem: this happens on some hosts when the
		   fw is not loaded. */
		err("get_op_mode() failed: %d", ret);
	}else{
		info("op_mode = %d", (int)dev->op_mode);
		if(dev->op_mode == 4){
			info("firmware already loaded");
			goto error; /* it'fine, but we are not needed */
		}
	}

	dbg("going for download");
	/* here our main action takes place: */
	ret = usbdfu_download(dev, drv->fw_buf, drv->fw_buf_len);
	if(ret < 0){
		err("download failed");
		goto error;
	}

	set_bit(KEVENT_FLAG_AGAIN, &dev->kevent_flags);
//	defer_kevent(dev, KEVENT_FLAG_RESET);
	
	init_timer(&dev->timer_reset);
        dev->timer_reset.data = (long) dev;
        dev->timer_reset.function = kevent_timer_reset;

	mod_timer(&dev->timer_reset, jiffies + 2*HZ);

	up(&dev->sem);
	goto exit;
	
error:
	up(&dev->sem);
	usbdfu_delete (dev);
	dev = NULL;

exit:
	up(&usbdfu_lock);

	dbg("usbdfu_probe() exiting");
	return dev;
}


static void usbdfu_disconnect(struct usb_device *udev, void *ptr)
{
	struct usbdfu *dev;

	dev = (struct usbdfu *)ptr;
	
	usbdfu_delete(dev);
		
	info("USB DFU now disconnected");
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

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

