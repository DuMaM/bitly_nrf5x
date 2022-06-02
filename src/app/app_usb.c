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
#include <drivers/uart.h>

void app_usb_init(void)
{
    int ret = 0;

#ifdef CONFIG_BOARD_PARTICLE_XENON

    ret = usb_enable(NULL);
    if (ret != 0)
    {
        printk("Failed to enable USB\n");
        return;
    }

    while (!device_is_ready(DEVICE_DT_GET(DT_CHOSEN(zephyr_console))))
    {
        k_sleep(K_MSEC(100));
        printk("CDC ACM device for console is not ready\n");
    }

    while (!device_is_ready(DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart))))
    {
        k_sleep(K_MSEC(100));
        printk("CDC ACM device for shell is not ready\n");
    }

    printk("Usb CDC ACM successfully enabled\n");
#endif

}
