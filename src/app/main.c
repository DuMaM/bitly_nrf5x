/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <app_usb.h>
#include <cmd.h>
#include <bt_test.h>

#include <sys/printk.h>
#include <spi_adc.h>

void main(void)
{
    app_usb_init();
#ifdef CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP
    ads129x_setup();
#endif
    bt_init();

    printk("\n");
    printk("Press button 1 on the master board.\n");
    printk("Press button 2 on the slave board.\n");
}
