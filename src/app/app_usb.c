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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <nrfx.h>
#include <app_usb.h>


LOG_MODULE_DECLARE(main);

void app_usb_init(void)
{
    int ret = 0;

#if DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_shell_uart), zephyr_cdc_acm_uart)

    while (!device_is_ready(DEVICE_DT_GET(DT_CHOSEN(zephyr_console))))
    {
        k_sleep(K_MSEC(100));
        //LOG_ERR("CDC ACM device for console is not ready");
    }

    while (!device_is_ready(DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart))))
    {
        k_sleep(K_MSEC(100));
        //LOG_ERR("CDC ACM device for shell is not ready");
    }

    ret = usb_enable(NULL);
    if (ret != 0)
    {
        //LOG_ERR("Failed to enable USB");
        return;
    }

    LOG_INF("Usb CDC ACM successfully enabled");
#endif
}
