/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <app_usb.h>
#include <cmd.h>
#include <bt_test.h>
#include <main.h>
#include <zephyr/drivers/hwinfo.h>

// logging
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

// work queue
K_THREAD_STACK_DEFINE(main_work_q_stack, MAIN_QUEUE_STACK_SIZE);
struct k_work_q main_work_q;

void main(void)
{
    /* init work queue */
    k_work_queue_init(&main_work_q);
    k_work_queue_start(&main_work_q, main_work_q_stack, K_THREAD_STACK_SIZEOF(main_work_q_stack), MAIN_QUEUE_PRIORITY, NULL);

    /* init pheriferials */
    app_usb_init();
    bt_init();

    uint32_t dev_id[2] = {0, 0};
    hwinfo_get_device_id((void*)dev_id, sizeof(dev_id));
    LOG_INF("Core ID: --%"PRIu32" %"PRIu32"--", dev_id[0], dev_id[1]);
}
