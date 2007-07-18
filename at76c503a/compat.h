/*
 * Compatibility code for older Linux kernels
 *
 * Copyright (c) 2007 Pavel Roskin <proski@gnu.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#define skb_reset_mac_header(_skb) \
	do { (_skb)->mac.raw = (_skb)->data; } while(0)
#define skb_end_pointer(_skb) ((_skb)->end)
#define skb_tail_pointer(_skb) ((_skb)->tail)
#endif
