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
#include <linux/skbuff.h>
static inline unsigned char *skb_mac_header(const struct sk_buff *skb)
{
	return skb->mac.raw;
}

static inline void skb_reset_mac_header(struct sk_buff *skb)
{
	skb->mac.raw = skb->data;
}

static inline void skb_set_mac_header(struct sk_buff *skb, const int offset)
{
	skb->mac.raw = skb->data + offset;
}

static inline unsigned char *skb_end_pointer(const struct sk_buff *skb)
{
	return skb->end;
}

static inline unsigned char *skb_tail_pointer(const struct sk_buff *skb)
{
	return skb->tail;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
#undef INIT_WORK
#define INIT_WORK(_work, _func)						\
	do {								\
		INIT_LIST_HEAD(&(_work)->entry);			\
		(_work)->pending = 0;					\
		PREPARE_WORK((_work), (void (*)(void *))(_func),	\
			     (void *)(_work));				\
		init_timer(&(_work)->timer);				\
	} while (0)
#endif
