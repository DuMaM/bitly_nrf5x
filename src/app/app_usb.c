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

#include <string.h>
#include <stdio.h>

#include <zephyr.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include <nrfx.h>
#include <app_usb.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(main);

#ifdef CONFIG_BOARD_PARTICLE_XENON

void app_usb_init(void)
{
    int ret = 0;

    ret = usb_enable(NULL);
    if (ret != 0)
    {
        LOG_ERR("Failed to enable USB");
        return;
    }

    while (!device_is_ready(DEVICE_DT_GET(DT_CHOSEN(zephyr_console))))
    {
        k_sleep(K_MSEC(100));
        LOG_ERR("CDC ACM device for console is not ready");
    }

    while (!device_is_ready(DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart))))
    {
        k_sleep(K_MSEC(100));
        LOG_ERR("CDC ACM device for shell is not ready");
    }

    LOG_INF("Usb CDC ACM successfully enabled");
}
#else
void app_usb_init(void)
{
}
#endif
