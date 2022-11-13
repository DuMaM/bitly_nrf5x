/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_DECLARE(main);

/* size of stack area used by each thread */
#define STACKSIZE 256

/* scheduling priority used by each thread */
#define PRIORITY 10

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

typedef struct _led_config
{
    struct gpio_dt_spec spec;
    const char *gpio_pin_name;
} led_config_t;

void blink_th(void *_missed)
{
    int ret;
    static led_config_t leds[] = {{
                                      .spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
                                      .gpio_pin_name = DT_PROP_OR(LED1_NODE, label, ""),
                                  },
                                  {
                                      .spec = GPIO_DT_SPEC_GET_OR(LED2_NODE, gpios, {0}),
                                      .gpio_pin_name = DT_PROP_OR(LED2_NODE, label, ""),
                                  },
                                  {
                                      .spec = GPIO_DT_SPEC_GET_OR(LED3_NODE, gpios, {0}),
                                      .gpio_pin_name = DT_PROP_OR(LED3_NODE, label, ""),
                                  }};

    for (int i = 0; i < 3; i++)
    {
        if (!device_is_ready(leds[i].spec.port))
        {
            LOG_ERR("Error: %s device is not ready\n", leds[i].spec.port->name);
            return;
        }

        ret = gpio_pin_configure_dt(&leds[i].spec, GPIO_OUTPUT);
        if (ret != 0)
        {
            LOG_ERR("Error %d: failed to configure pin %d (LED '%s')", ret, leds[i].spec.pin, leds[i].gpio_pin_name);
            return;
        }
    }

    while (1)
    {
        for (int i = 0; i < 3; i++)
        {
            gpio_pin_toggle(leds[i].spec.port, leds[i].spec.pin);
            k_msleep(330);
        }
    }
}

K_THREAD_DEFINE(blink_thread, STACKSIZE, blink_th, NULL, NULL, NULL, PRIORITY, 0, 1000);
