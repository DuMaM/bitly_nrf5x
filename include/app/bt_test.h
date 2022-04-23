#ifndef _BT_TEST_H_
#define _BT_TEST_H_

#include <bluetooth/bluetooth.h>
#include <bluetooth/crypto.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <performance_test.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

#include <shell/shell.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define PERF_TEST_CONFIG_TIMEOUT K_SECONDS(20)

void bt_init(void);
struct bt_conn *getSettings(void);
volatile bool isTestReady(void);

void scan_start(void);
void adv_start(void);

int connection_configuration_set(const struct shell *shell,
                                 const struct bt_le_conn_param *conn_param,
                                 const struct bt_conn_le_phy_param *phy,
                                 const struct bt_conn_le_data_len_param *data_len);

extern struct bt_performance_test performance_test;
extern struct k_sem performance_test_sem;
#endif /* _BT_TEST_H_ */
