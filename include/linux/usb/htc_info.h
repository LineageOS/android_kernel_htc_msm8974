/*
 * Copyright (C) 2011 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __HTC_INFO__
#define __HTC_INFO__

#include <mach/board.h>
struct usb_info {
	int *phy_init_seq;
	void (*phy_reset)(void);
	void (*hw_reset)(bool en);
	void (*usb_uart_switch)(int);
	void (*serial_debug_gpios)(int);
	void (*usb_hub_enable)(bool);
	int (*china_ac_detect)(void);
	void (*disable_usb_charger)(void);
	void (*change_phy_voltage)(int);
	int (*ldo_init) (int init);
	int (*ldo_enable) (int enable);
	void (*usb_mhl_switch)(bool);

	/* for notification when USB is connected or disconnected */
	int connect_type_ready;
	void (*usb_connected)(int);

	/* TODO: Dyson porting */
	#if 0
	enum usb_connect_type connect_type;
	#endif
	struct delayed_work chg_stop;
};

extern ssize_t otg_show_usb_phy_setting(char *buf);
extern ssize_t otg_store_usb_phy_setting(const char *buf, size_t count);

extern int usb_get_connect_type(void);
extern int msm_usb_get_connect_type(void);
extern int android_switch_function(unsigned func);
extern int android_show_function(char *buf);
extern void android_set_serialno(char *serialno);
extern void android_force_reset(void);
extern int htc_usb_enable_function(char *name, int ebl);

#define ANDROID_USB_ENABLE_FUNC(dev, conf, func) 		\
	if (android_enable_function(dev, conf, func)) {		\
		pr_err("android_usb: Cannot enable %s", func);	\
	}

#endif /* __HTC_INFO__ */

