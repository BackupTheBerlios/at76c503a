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
 *
 */

int usbdfu_register(struct usb_driver *driver, u8 *fw_buf, int fw_buf_len);
void usbdfu_deregister(struct usb_driver *driver);

int dfu_download(struct usb_device *udev, unsigned char *dfu_buffer,
		 unsigned int dfu_len);
