/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED2_NODE, okay)
#error "Unsupported board: led2 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED3_NODE, okay)
#error "Unsupported board: led2 devicetree alias is not defined"
#endif

struct led {
    struct gpio_dt_spec spec;
    const char *gpio_pin_name;
};


static const struct led led1 = {
    .spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
    .gpio_pin_name = DT_PROP_OR(LED1_NODE, label, ""),
};

static const struct led led2 = {
    .spec = GPIO_DT_SPEC_GET_OR(LED2_NODE, gpios, {0}),
    .gpio_pin_name = DT_PROP_OR(LED2_NODE, label, ""),
};

static const struct led led3 = {
    .spec = GPIO_DT_SPEC_GET_OR(LED3_NODE, gpios, {0}),
    .gpio_pin_name = DT_PROP_OR(LED3_NODE, label, ""),
};

void blink(const struct led *led, uint32_t sleep_ms)
{
    const struct gpio_dt_spec *spec = &led->spec;
    int cnt = 0;
    int ret;

    if (!device_is_ready(spec->port)) {
        printk("Error: %s device is not ready\n", spec->port->name);
        return;
    }

    ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure pin %d (LED '%s')\n", ret, spec->pin, led->gpio_pin_name);
        return;
    }

    while (1) {
        gpio_pin_set(spec->port, spec->pin, cnt % 2);
        k_msleep(sleep_ms);
        cnt++;
    }
}

#define LED_BLINK_SLEEP_MS 900
#define LED1_BLINK_OFFSET_MS 0
#define LED2_BLINK_OFFSET_MS (uint32_t)(LED_BLINK_SLEEP_MS/3)
#define LED3_BLINK_OFFSET_MS (uint32_t)(LED_BLINK_SLEEP_MS/3*2)

void blink_th(void *ledx)
{
    blink(ledx, LED_BLINK_SLEEP_MS);
}

K_THREAD_DEFINE(blink1_id, STACKSIZE, blink_th, &led1, NULL, NULL, PRIORITY, 0, LED1_BLINK_OFFSET_MS);
K_THREAD_DEFINE(blink2_id, STACKSIZE, blink_th, &led2, NULL, NULL, PRIORITY, 0, LED2_BLINK_OFFSET_MS);
K_THREAD_DEFINE(blink3_id, STACKSIZE, blink_th, &led3, NULL, NULL, PRIORITY, 0, LED3_BLINK_OFFSET_MS);