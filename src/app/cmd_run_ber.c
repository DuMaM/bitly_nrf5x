#include <zephyr/types.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/crypto.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <performance_test.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>

extern test_params_t test_params;
extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;

int test_run_ber(const struct shell *shell,
                             const uint16_t period_sec,
                             const struct bt_le_conn_param *conn_param,
                             const struct bt_conn_le_phy_param *phy,
                             const struct bt_conn_le_data_len_param *data_len,
                             const uint8_t pattern)
{
    int64_t stamp = 0;
    int64_t delta = 0;
    int64_t data = 0;
    int err;

    err = test_init(shell, conn_param, phy, data_len);
    if (err)
    {
        shell_error(shell, "Init test failed (err %d)", err);
        return err;
    }

    for (int i = 0; i < test_data_buffer_size; i++)
    {
        test_data_buffer[i] = pattern;
    }

    /* get cycle stamp */
    stamp = k_uptime_get_32();

    while (delta < period_sec)
    {
        err = bt_performance_test_write(&performance_test, test_data_buffer, test_data_buffer_size);
        if (err)
        {
            shell_error(shell, "GATT write failed (err %d)", err);
            break;
        }

        /* print graphics */
        printk("Sending %d: 11001100 ...\n", test_data_buffer_size);
        delta = k_uptime_delta(&stamp);
        data += test_data_buffer_size;
    }

    printk("\nDone\n");
    printk("[local] sent %"PRIi64" bytes (%"PRIi64" KB) in %"PRIi64" ms at %"PRIu64" kbps\n", data, data / 1024, delta, ((uint64_t)data * 8 / delta));

    /* read back char from peer */
    err = bt_performance_test_read(&performance_test);
    if (err)
    {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }

    k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);

    return 0;
}

int test_run_ber_alternating_cmd(const struct shell *shell, size_t argc, char **argv)
{

    uint16_t period_sec = 0;

    if (argc <= 1)
    {
        shell_help(shell);
        return SHELL_CMD_HELP_PRINTED;
    }

    if (argc > 2)
    {
        shell_error(shell, "%s: bad parameters count", argv[0]);
        return -EINVAL;
    }

    period_sec = strtol(argv[1], NULL, 10);

#define MAX_TEST_SIZE 1800
    if (period_sec > MAX_TEST_SIZE)
    {
        shell_error(shell, "%s: Invalid setting: %d", argv[0], period_sec);
        shell_error(shell, "Test time must be lower then: %d", MAX_TEST_SIZE);
        return -EINVAL;
    }
    shell_print(shell, "Test time set to: %d min", period_sec / 60);

    return test_run_ber(shell,
                        period_sec,
                        test_params.conn_param,
                        test_params.phy,
                        test_params.data_len,
                        0x33);
}
