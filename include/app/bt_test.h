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
void scan_start(struct k_work *item);
void adv_start(struct k_work *item);

int connection_configuration_set(const struct bt_le_conn_param *conn_param,
                                 const struct bt_conn_le_phy_param *phy,
                                 const struct bt_conn_le_data_len_param *data_len);

extern struct bt_performance_test performance_test;
extern struct k_sem performance_test_sem;
#endif /* _BT_TEST_H_ */
