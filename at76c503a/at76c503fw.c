/*
 * USB at76c503 firmware loader
 *
 * Copyright (c) 2002 - 2003 Oliver Kurth <oku@masqmail.cx>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation; either version 2 of
 *	the License, or (at your option) any later version.
 *
 *
 */

/* This code is _very_ hackish, I know that. It was written with trial
   and error, and will be made obsolete, when the download will be
   done in the module (oku) */

/*
 * See here for the DFU specifications:
 * http://www.usb.org/developers/devclass_docs/usbdfu10.pdf
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <usb.h>

typedef unsigned char UCHAR;
#include "internalr.h"
#include "externalr.h"

#define DEVICE_VENDOR_REQUEST_OUT    0x40
#define DEVICE_VENDOR_REQUEST_IN     0xc0
#define INTERFACE_VENDOR_REQUEST_OUT 0x41
#define INTERFACE_VENDOR_REQUEST_IN  0xc1
#define CLASS_REQUEST_OUT            0x21
#define CLASS_REQUEST_IN             0xa1

/* Define these values to match your device */
#define USB_BELKIN_VENDOR_ID	0x0d5c
#define USB_BELKIN_PRODUCT_ID	0xa002

#define TIMEOUT 1000 /* ms */

struct hwcfg_rfmd {
  unsigned char CR20Values[14]; 
  unsigned char CR21Values[14]; 
  unsigned char BB_CR[14]; 
  unsigned char PIDVID[4]; 
  unsigned char mac_addr[6]; 
  unsigned char mRegulatoryDomain; 
  unsigned char LowPowerValues[14];     
  unsigned char NormalPowerVlues[14]; 
  unsigned char Reserved1[3];   
};

static inline
int set_remap(struct usb_dev_handle *devh)
{
  return usb_control_msg(devh,
			 INTERFACE_VENDOR_REQUEST_OUT, 0x0a,
			 0, 0,
			 NULL, 0, TIMEOUT);
}

static inline
int get_op_mode(struct usb_dev_handle *devh, unsigned char *mode)
{
  return usb_control_msg(devh,
			 INTERFACE_VENDOR_REQUEST_IN, 0x33,
			 0x01, 0,
			 mode, 1, TIMEOUT);
}

static inline
int get_dfu_state(struct usb_dev_handle *devh, unsigned char *state)
{
  return usb_control_msg(devh,
			 CLASS_REQUEST_IN, 0x05,
			 0, 0,
			 state, 1, TIMEOUT);
}

static inline
int get_dfu_status(struct usb_dev_handle *devh, unsigned char *state)
{
  return usb_control_msg(devh,
			 CLASS_REQUEST_IN, 0x03,
			 0, 0,
			 state, 6, TIMEOUT);
}

static inline
int set_dfu_block_int(struct usb_dev_handle *devh,
		  int i, unsigned char *buf, int bsize)
{
  return usb_control_msg(devh,
			 CLASS_REQUEST_OUT, 0x01,
			 i, 0,
			 buf, bsize, TIMEOUT);
}
  
static inline
int set_dfu_block_ext(struct usb_dev_handle *devh,
		      int i, unsigned char *buf, int bsize)
{
  return usb_control_msg(devh,
			 DEVICE_VENDOR_REQUEST_OUT, 0x0e,
			 0x0802, i,
			 buf, bsize, TIMEOUT);
}

static inline
int get_hw_cfg(struct usb_dev_handle *devh,
	       unsigned char *buf, int buf_size)
{
    return usb_control_msg(devh,
			   INTERFACE_VENDOR_REQUEST_IN, 0x33,
			   ((0x0a << 8) | 0x02), 0,
			   buf, buf_size, TIMEOUT);
}


static int fw_download_int(struct usb_dev_handle *devh,
			   unsigned char *realbuf, int size)
{
  int i = 0;
  unsigned char *dfu_status = malloc (6);
  unsigned char *dfu_state = malloc(1);
  unsigned char *buf = malloc (size);

  if(!buf)
    return -ENOMEM;

  memcpy(buf, realbuf, size);

  //  while(size > 0){
  while(1){
    int bsize = size > 1024 ? 1024 : size;
    int ret;

    do{
      if((ret = get_dfu_state(devh, dfu_state)) < 0){
	err("get_dfu 1 failed: %d", ret);
	return ret;
      }

      if((*dfu_state != 5) && (*dfu_state != 6) && (*dfu_state != 3) && (*dfu_state != 2)){
	/* TODO: fix this */
	err("cannot handle dfu_state = %d", *dfu_state);
	return -EIO;
      }
      printf("dfu_state 1 = %d", *dfu_state);

      /*
      if(*dfu_state == 6){
      }
      */

      if((*dfu_state == 3 || *dfu_state == 6)){
	if((ret = get_dfu_status(devh, dfu_status)) < 0){
	  err("get_dfu 2 failed: %d", ret);
	  return ret;
	}
	*dfu_state = dfu_status[4]; /* what an ugly hardware... */

	if(*dfu_state == 7){
	  return 0;
	}
	
	if((*dfu_state != 5) && (*dfu_state != 6) && (*dfu_state != 3) &&
	   (*dfu_state != 2) && (*dfu_state != 7)){
	  /* TODO: fix this */
	  err("cannot handle dfu_state(2) = %d", *dfu_state);
	  return -EIO;
	}
      }
      //    }while((dfu_state != 2) && (dfu_state != 5) && (dfu_state != 6));
    }while((*dfu_state != 2) && (*dfu_state != 5));

    printf("fw_int, size = %d, bsize = %d, i = %d, buf = %p\n", size, bsize, i, buf);
    if((ret = set_dfu_block_int(devh, i, buf, bsize)) < 0){
      err("fw_int failed: %d, i = %d", ret, i);
      return ret;
    }
    buf += bsize;
    size -= bsize;
    i++;
  }
  free(dfu_status);
  free(dfu_state);
  free(buf);
  return 0;
}

static int fw_download_ext(struct usb_dev_handle *devh, unsigned char *buf, int size)
{
  int i = 0, ret;
  unsigned char dev_mode;
  unsigned char *tmpbuf = malloc(1024);

  if((ret = get_op_mode(devh, &dev_mode)) < 0){
    fprintf(stderr, "get_op_mode failed, ret = %d", ret);
  }

  if(dev_mode != 4){
    err("dev_mode = %d != 4, that's bad", dev_mode);
  }

  while(size > 0){
    int bsize = size > 1024 ? 1024 : size;
    int ret;

    memcpy(tmpbuf, buf, bsize);
    printf("fw_ext, size = %d, bsize = %d, i = %d, buf = %p\n", size, bsize, i, buf);
    if((ret = set_dfu_block_ext(devh, i, tmpbuf, bsize)) < 0){
      err("fw_ext failed: %d, i = %d", ret, i);
      return ret;
    }
    buf += bsize;
    size -= bsize;
    i++;
  }
  if((ret = set_dfu_block_ext(devh, i, tmpbuf, 0)) < 0){
    err("fw_ext failed: %d, i = %d", ret, i);
    return ret;
  }
  free(tmpbuf);
  return 0;
}

int main(int argc, char *argv[])
{
  struct usb_bus *bus;
  struct usb_device *dev;
  int do_int = 0, do_ext = 0, c;

  usb_init();
  usb_set_debug(0);

  while(1){
    c = getopt(argc, argv, "ie");

    if(c == -1)
      break;

    switch(c){
    case 'i':
      do_int = 1;
      break;
    case 'e':
      do_ext = 1;
      break;
    case '?':
      break;
    }
  }

  usb_find_busses();
  usb_find_devices();

  printf("bus/device  idVendor/idProduct\n");
  for (bus = usb_busses; bus; bus = bus->next) {
    for (dev = bus->devices; dev; dev = dev->next) {

      if ((dev->descriptor.idVendor == USB_BELKIN_VENDOR_ID) &&
	  (dev->descriptor.idProduct == USB_BELKIN_PRODUCT_ID)) {
	printf("Belkin USB Adapter found\n");
	goto endloop;
      }

      printf("%s/%s     %04X/%04X\n", bus->dirname, dev->filename,
        dev->descriptor.idVendor, dev->descriptor.idProduct);

      if (!dev->config) {
        fprintf(stderr, "Couldn't retrieve descriptors\n");
        continue;
      }
    }
  }

 endloop:
  if(bus && dev){
    struct usb_dev_handle *devh = usb_open(dev);
    int ret;
    unsigned char *dummy = malloc(1);

    if(!devh){
      fprintf(stderr, " could not open dev: %s\n", strerror(errno));
      exit(1);
    }

    ret = usb_claim_interface(devh, 0);
    if(ret < 0){
      fprintf(stderr, "usb_claim_interface failed, ret = %d\n", ret);
    }

    if(do_int){
      ret = get_op_mode(devh, dummy);
      if(ret < 0){
	fprintf(stderr, "get_op_mode failed, ret = %d (no problem)", ret);
      }

      printf("download int\n");
      ret = fw_download_int(devh, InternalRFMD, sizeof(InternalRFMD));
      if(ret < 0){
	fprintf(stderr, "download int failed:  %s\n", strerror(errno));
	goto close;
      }
      
      sleep(1);

      printf("remap\n");
      ret = set_remap(devh);
      if(ret < 0){
	fprintf(stderr, "remap failed:  %s\n", strerror(errno));
	goto close;
      }

      sleep(2);

      /*
	usb_close(devh);
	usb_init();
	devh = usb_open(dev);
      */
      
      printf("reset\n");
      ret = usb_reset(devh);
      if(ret < 0){
	fprintf(stderr, "reset failed:  %s\n", strerror(errno));
	goto close;
      }

    }

    if(do_ext){
      usb_set_configuration(devh, 1);
      if(ret < 0){
	fprintf(stderr, "usb_set_configuration failed:  %s\n", strerror(errno));
	//	goto close;
      }

      printf("download ext\n");
      ret = fw_download_ext(devh, ExternalRFMD, sizeof(ExternalRFMD));
      if(ret < 0){
	fprintf(stderr, "download ext failed:  %s\n", strerror(errno));
	goto close;
      }
    }
close:
    usb_release_interface(devh, 0);
    usb_close(devh);

  }else{
    printf("Belkin USB Adapter NOT found\n");
    exit(1);
  }

  return 0;
}
