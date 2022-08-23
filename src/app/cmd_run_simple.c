#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <performance_test.h>
#include "img_file.h"

extern test_params_t test_params;
extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;

LOG_MODULE_DECLARE(main);

void print_2d_array(uint8_t *num, uint8_t size)
{
    int counter = 0;
    while (counter++ < size)
    {
        printk("%c", *num);
        num++;
    }
}

int test_run(const struct shell *shell,
             const struct bt_le_conn_param *conn_param,
             const struct bt_conn_le_phy_param *phy,
             const struct bt_conn_le_data_len_param *data_len)
{
    int64_t stamp;
    int64_t delta;
    uint32_t prog = 0;
    uint32_t data = 0;
    int err;

    err = test_init(shell, conn_param, phy, data_len, BT_TEST_TYPE_SIMPLE);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return err;
    }

    /* get cycle stamp */
    stamp = k_uptime_get_32();
    uint8_t buffer_size = test_data_buffer_size;
    uint8_t *img_prt = NULL;
    while (prog < IMG_SIZE)
    {
        img_prt = ((uint8_t *)img) + prog;
        if (IMG_SIZE - prog > test_data_buffer_size)
        {
            buffer_size = test_data_buffer_size;
        }
        else
        {
            buffer_size = IMG_SIZE - prog;
        }

        err = bt_performance_test_write(&performance_test, img_prt, buffer_size);
        if (err)
        {
            LOG_ERR("GATT write failed (err %d)", err);
            break;
        }

        print_2d_array(img_prt, buffer_size);

        img_prt += prog;
        prog += buffer_size;
        data += buffer_size;


    }

    delta = k_uptime_delta(&stamp);

    LOG_INF("Done");
    LOG_INF("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps", data, data / 1024, delta, ((uint64_t)data * 8 / delta));

    /* read back char from peer */
    err = bt_performance_test_read(&performance_test);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return err;
    }

    k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);

    instruction_print();

    return 0;
}

int test_run_cmd(const struct shell *shell, size_t argc, char **argv)
{
    return test_run(shell, test_params.conn_param, test_params.phy, test_params.data_len);
}
