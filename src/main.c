/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <app_usb.h>
#include <cmds.h>
#include <pio.h>
#include <bt_test.h>

#include <sys/printk.h>

void main(void)
{
    bt_init();
    app_usb_init();
    buttons_init();

    printk("\n");
    printk("Press button 1 on the master board.\n");
    printk("Press button 2 on the slave board.\n");
}
