#ifndef _CMD_H_
#define _CMD_H_

#include <zephyr/bluetooth/conn.h>
#include <zephyr/shell/shell.h>
#include <zephyr/types.h>

#include <errno.h>
#include <stdlib.h>

#define INTERVAL_MIN 28 /* 28 units, 35 ms */
#define INTERVAL_MAX 28 /* 28 units, 35 ms */
#define CONN_LATENCY 0
#define UNIT_SCALER 1.25

#define MIN_CONN_INTERVAL 6
#define MAX_CONN_INTERVAL 3200
#define SUPERVISION_TIMEOUT 1000

#define SHELL_TEST_RUN_PRIO 7

typedef struct
{
    struct bt_le_conn_param *conn_param;
    struct bt_conn_le_phy_param *phy;
    struct bt_conn_le_data_len_param *data_len;
    bool enable_rssi;
    bool fit_buffer;
} test_params_t;
extern test_params_t test_params;

void instruction_print(void);
int default_cmd(const struct shell *shell, size_t argc, char **argv);
const char *phy_str(const struct bt_conn_le_phy_param *phy);


#endif /* _CMD_H_ */
