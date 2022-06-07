#ifndef _CMD_RUN_H_
#define _CMD_RUN_H_

#include <shell/shell.h>
#include <bluetooth/conn.h>

int test_init(const struct shell *shell,
              const struct bt_le_conn_param *conn_param,
              const struct bt_conn_le_phy_param *phy,
              const struct bt_conn_le_data_len_param *data_len);

int test_run_ber_alternating_cmd(const struct shell *shell, size_t argc, char **argv);
int test_run_ber_oppsed_cmd(const struct shell *shell, size_t argc, char **argv);
int test_run_cmd(const struct shell *shell, size_t argc, char **argv);

#endif // _CMD_RUN_H_