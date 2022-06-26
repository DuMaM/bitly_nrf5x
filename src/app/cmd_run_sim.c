#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>

#include <performance_test.h>
#include "sim_file.h"

extern test_params_t test_params;
extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;

int sim_run(const struct shell *shell,
             const struct bt_le_conn_param *conn_param,
             const struct bt_conn_le_phy_param *phy,
             const struct bt_conn_le_data_len_param *data_len)
{
    int64_t stamp;
    int64_t delta;
    uint32_t prog = 0;
    uint32_t data = 0;
    int err;

    err = test_init(shell, conn_param, phy, data_len);
    if (err)
    {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }

    /* get cycle stamp */
    stamp = k_uptime_get_32();
    while (prog < 1000*12)
    {
        for (int i = 0; i < test_data_buffer_size; i++)
        {
            test_data_buffer[i] = sim_data[prog][i];
        }
        err = bt_performance_test_write(&performance_test, test_data_buffer, test_data_buffer_size);
        if (err)
        {
            shell_error(shell, "GATT write failed (err %d)", err);
            break;
        }

        /* print graphics */
        printk("%f", sim_data+prog);
        data += test_data_buffer_size;
        prog++;
    }

    delta = k_uptime_delta(&stamp);

    shell_print(shell, "\nDone\n");
    shell_print(shell, "[local] sent %u bytes (%u KB) in %lld ms at %llu kbps\n", data, data / 1024, delta, ((uint64_t)data * 8 / delta));

    /* read back char from peer */
    err = bt_performance_test_read(&performance_test);
    if (err)
    {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }

    k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);

    instruction_print();

    return 0;
}

int sim_run_cmd(const struct shell *shell, size_t argc, char **argv)
{
    return sim_run(shell, test_params.conn_param, test_params.phy, test_params.data_len);
}
