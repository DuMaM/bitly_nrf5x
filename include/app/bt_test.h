#ifndef _BT_TEST_H_
#define _BT_TEST_H_

#include <zephyr/shell/shell.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define PERF_TEST_CONFIG_TIMEOUT K_SECONDS(20)

void bt_init(void);
struct bt_conn *getSettings(void);
volatile bool isTestReady(void);
void restore_state();
void scan_start();
void adv_start();

int connection_configuration_set();
void config_update_param();
void config_update_len();
void config_update_phy();

extern struct bt_performance_test performance_test;
#endif /* _BT_TEST_H_ */
