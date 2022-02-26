#ifndef _CMDS_H_
#define _CMDS_H_

#include <bluetooth/conn.h>

#include <errno.h>
#include <shell/shell.h>
#include <zephyr/types.h>

#include <stdlib.h>

#define INTERVAL_MIN 0x140 /* 320 units, 400 ms */
#define INTERVAL_MAX 0x140 /* 320 units, 400 ms */
#define CONN_LATENCY 0

#define MIN_CONN_INTERVAL   6
#define MAX_CONN_INTERVAL   3200
#define SUPERVISION_TIMEOUT 1000

enum CONNECTION_MODE {
	CONNECTION_MASTER,
	CONNECTION_SLAVE
};
static struct test_params {
	struct bt_le_conn_param *conn_param;
	struct bt_conn_le_phy_param *phy;
	struct bt_conn_le_data_len_param *data_len;
	bool   connection_mode;
} test_params = {
	.conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, CONN_LATENCY,
				       SUPERVISION_TIMEOUT),
	.phy = BT_CONN_LE_PHY_PARAM_2M,
	.data_len = BT_LE_DATA_LEN_PARAM_MAX
};

#endif /* _CMDS_H_ */