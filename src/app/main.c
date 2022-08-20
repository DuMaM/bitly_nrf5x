/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <app_usb.h>
#include <cmd.h>
#include <bt_test.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

void main(void)
{
    app_usb_init();
    bt_init();

    LOG_INF("\n");
    LOG_INF("Press button 1 on the master board.\n");
    LOG_INF("Press button 2 on the slave board.\n");

}
