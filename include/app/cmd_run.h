#ifndef _CMD_RUN_H_
#define _CMD_RUN_H_

#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/conn.h>
#include <performance_test.h>

int test_init(const struct shell *shell,
              const struct bt_le_conn_param *conn_param,
              const struct bt_conn_le_phy_param *phy,
              const struct bt_conn_le_data_len_param *data_len,
              const bt_test_type_t type);

int test_run_ber_alternating_cmd(const struct shell *shell, size_t argc, char **argv);
int test_run_ber_oppsed_cmd(const struct shell *shell, size_t argc, char **argv);
int sim_run_cmd(const struct shell *shell, size_t argc, char **argv);
int test_run_cmd(const struct shell *shell, size_t argc, char **argv);

#endif // _CMD_RUN_H_