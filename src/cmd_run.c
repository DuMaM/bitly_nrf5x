#include <cmds.h>
#include <bt_test.h>
#include "img_file.h"

extern test_params_t test_params;

int test_run(const struct shell *shell,
         const struct bt_le_conn_param *conn_param,
         const struct bt_conn_le_phy_param *phy,
         const struct bt_conn_le_data_len_param *data_len)
{
    int err;
    uint64_t stamp;
    int64_t delta;
    uint32_t data = 0;
    uint32_t prog = 0;

    /* a dummy data buffer */
    static char dummy[256];

    if (!getSettings()) {
        shell_error(shell, "Device is disconnected %s",
                "Connect to the peer device before running test");
        return -EFAULT;
    }

    if (!isTestReady()) {
        shell_error(shell, "Device is not ready."
            "Please wait for the service discovery and MTU exchange end");
        return 0;
    }

    shell_print(shell, "\n==== Starting throughput test ====");

    err = connection_configuration_set(shell, conn_param, phy, data_len);
    if (err) {
        return err;
    }

    /* reset peer metrics */
    err = bt_throughput_write(&throughput, dummy, 1);
    if (err) {
        shell_error(shell, "Reset peer metrics failed.");
        return err;
    }

    /* Make sure that all BLE procedures are finished. */
    k_sleep(K_MSEC(500));

    /* get cycle stamp */
    stamp = k_uptime_get_32();

    while (prog < IMG_SIZE) {
        err = bt_throughput_write(&throughput, dummy, 244);
        if (err) {
            shell_error(shell, "GATT write failed (err %d)", err);
            break;
        }

        /* print graphics */
        printk("%c", img[prog / IMG_X][prog % IMG_X]);
        data += 244;
        prog++;
    }

    delta = k_uptime_delta(&stamp);

    printk("\nDone\n");
    printk("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps\n",
           data, data / 1024, delta, ((uint64_t)data * 8 / delta));

    /* read back char from peer */
    err = bt_throughput_read(&throughput);
    if (err) {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }

    k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);

    instruction_print();

    return 0;
}


static int test_run_cmd(const struct shell *shell, size_t argc,
            char **argv)
{
    return test_run(shell, test_params.conn_param, test_params.phy,
            test_params.data_len);
}

SHELL_CMD_REGISTER(run, NULL, "Run the test", test_run_cmd);
