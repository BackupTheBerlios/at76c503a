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
 *
 */

#ifndef _USBDFU_H
#define _USBDFU_H

#include <linux/usb.h>

struct usbdfu_info {
	const char *name;
	const struct usb_device_id *id_table;
	u8 *fw_buf;
	int fw_buf_len;
	int flags;
	int (*pre_download_hook)(struct usb_device *udev);
	int (*post_download_hook)(struct usb_device *udev);
	unsigned int reset_delay;
};

int usbdfu_register(struct usbdfu_info *info);
void usbdfu_deregister(struct usbdfu_info *info);
//int usbdfu_register(struct usb_driver *driver, u8 *fw_buf, int fw_buf_len);
//void usbdfu_deregister(struct usb_driver *driver);

int usbdfu_in_use(struct usb_device *udev, unsigned int ifnum);

int usbdfu_initiate_download(struct usb_device *udev);

#endif /* _USBDFU_H */
