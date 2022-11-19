/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <app_usb.h>
#include <cmd.h>
#include <bt_test.h>
#include <main.h>

// logging
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

void main(void)
{
    /* init pheriferials */
    app_usb_init();
    bt_init();
}
