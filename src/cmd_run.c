#include <cmds.h>

extern test_params_t test_params;

extern int test_run(const struct shell *shell,
            const struct bt_le_conn_param *conn_param,
            const struct bt_conn_le_phy_param *phy,
            const struct bt_conn_le_data_len_param *data_len);

static int test_run_cmd(const struct shell *shell, size_t argc,
            char **argv)
{
    return test_run(shell, test_params.conn_param, test_params.phy,
            test_params.data_len);
}

SHELL_CMD_REGISTER(run, NULL, "Run the test", test_run_cmd);
