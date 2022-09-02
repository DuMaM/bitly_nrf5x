#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

#include <cmd_run.h>
#include <cmd.h>
#include <performance_test.h>

#include <bt_test.h>

#define MAX_TEST_SIZE (1800 * 1000)

extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;


char *bitString(uint8_t n, char* out_bin_string)
{
    int i;
    for (i = 7; i >= 0; i--)
    {
        out_bin_string[i] = (n & 1) + '0';
        n >>= 1;
    }
    out_bin_string[8] = '\0';
    return out_bin_string;
}

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
    uint64_t prog = 0;
    char pattern_string[7];
    int err;

    err = test_init(conn_param, phy, data_len, BT_TEST_TYPE_BER);
    if (err)
    {
        shell_error(shell, "Init test failed (err %d)", err);
        return err;
    }

    for (int i = 0; i < test_data_buffer_size; i++)
    {
        test_data_buffer[i] = pattern;
    }

    bitString(pattern, pattern_string);

    /* get cycle stamp */
    stamp = k_uptime_get_32();

    while (delta/1000 < period_sec)
    {
        err = bt_performance_test_write(&performance_test, test_data_buffer, test_data_buffer_size);
        if (err)
        {
            shell_error(shell, "GATT write failed (err %d)", err);
            break;
        }

        /* print data */
        prog++;
        shell_print(shell, "(%"PRIu64")Sending %"PRIu64": %s ...", prog, delta, pattern_string);
        data += test_data_buffer_size;
        delta += k_uptime_delta(&stamp);
    }

    shell_print(shell, "Done");
    shell_print(shell, "[local] sent %"PRIi64" bytes (%"PRIi64" KB) in %"PRIi64" ms at %"PRIu64" kbps", data, data / 1024, delta, ((data * 8) / delta));

    /* read back char from peer and wait to finish it */
    err = bt_performance_test_read(&performance_test);
    if (err)
    {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }

    k_sem_take(&cmd_sync_sem, PERF_TEST_CONFIG_TIMEOUT);
    shell_print(shell, "Command finished");

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

    if (period_sec > MAX_TEST_SIZE)
    {
        shell_error(shell, "%s: Invalid setting: %d", argv[0], period_sec);
        shell_error(shell, "Test time must be lower then: %d", MAX_TEST_SIZE);
        return -EINVAL;
    }
    shell_print(shell, "Test time set to: %df min", period_sec / 60);

    return test_run_ber(shell,
                        period_sec,
                        test_params.conn_param,
                        test_params.phy,
                        test_params.data_len,
                        0x33);
}


int test_run_ber_oppsed_cmd(const struct shell *shell, size_t argc, char **argv)
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

    if (period_sec > MAX_TEST_SIZE)
    {
        shell_error(shell, "%s: Invalid setting: %d", argv[0], period_sec);
        shell_error(shell, "Test time must be lower then: %d", MAX_TEST_SIZE);
        return -EINVAL;
    }
    shell_print(shell, "Test time set to: %d min", period_sec / 60 / 1000);

    return test_run_ber(shell,
                        period_sec,
                        test_params.conn_param,
                        test_params.phy,
                        test_params.data_len,
                        0xAA);
}
