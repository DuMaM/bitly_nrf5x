/**
 * @file
 * @brief Sample echo app for CDC ACM class
 *
 * Sample app for USB CDC ACM class driver. The received data is echoed back
 * to the serial port.
 */

#include <device.h>
#include <zephyr.h>

#include <usb/usb_device.h>

/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample echo app for CDC ACM class
 *
 * Sample app for USB CDC ACM class driver. The received data is echoed back
 * to the serial port.
 */

#include <app_usb.h>
#include <stdio.h>
#include <string.h>
#include <device.h>
#include <zephyr.h>
#include <nrfx.h>
#include <usb/usb_device.h>
#include <sys/printk.h>

void app_usb_init(void)
{
#ifdef CONFIG_BOARD_PARTICLE_XENON

	const struct device *dev;
	int ret = 0;

	dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (!device_is_ready(dev)) {
		printk("CDC ACM device not ready");
		return;
	}

	ret = usb_enable(NULL);
	if (ret != 0) {
		printk("Failed to enable USB");
		return;
	}

#endif
}
