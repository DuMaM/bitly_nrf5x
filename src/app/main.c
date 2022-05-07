/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <app_usb.h>
#include <cmds.h>
#include <bt_test.h>

#include <sys/printk.h>

void main(void)
{
    app_usb_init();
    bt_init();

    printk("\n");
    printk("Press button 1 on the master board.\n");
    printk("Press button 2 on the slave board.\n");
}
