/*
 *  Copyright (C) 2012 Samsung Electronics
 *  Lukasz Majewski <l.majewski@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __G_DOWNLOAD_H_
#define __G_DOWNLOAD_H_

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <linker_lists.h>

extern int stop_fastboot;
/*
 * @usb_fname: unescaped USB function name
 * @callback_ptr: bind callback, one per function name
 */
#define DECLARE_GADGET_BIND_CALLBACK(usb_fname, callback_ptr) \
	ll_entry_declare(struct g_dnl_bind_callback, \
			__usb_function_name_##usb_fname, \
			g_dnl_bind_callbacks) = { \
				.usb_function_name = #usb_fname, \
				.fptr = callback_ptr \
			}

typedef int (*g_dnl_bind_callback_f)(struct usb_configuration *);

/* used in Gadget downloader callback linker list */
struct g_dnl_bind_callback {
	const char *usb_function_name;
	g_dnl_bind_callback_f fptr;
};

int g_dnl_bind_fixup(struct usb_device_descriptor *, const char *);
int g_dnl_board_usb_cable_connected(void);
int g_dnl_register(const char *s);
void g_dnl_unregister(void);
void g_dnl_set_serialnumber(char *);

#endif /* __G_DOWNLOAD_H_ */