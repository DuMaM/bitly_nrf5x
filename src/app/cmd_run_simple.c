#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>
#include <main.h>

#include <zephyr/logging/log.h>

#include <performance_test.h>
#include "img_file.h"

extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;

LOG_MODULE_DECLARE(main);

static void test_run(struct k_work *item)
{
    const struct bt_le_conn_param *conn_param = test_params.conn_param;
    const struct bt_conn_le_phy_param *phy = test_params.phy;
    const struct bt_conn_le_data_len_param *data_len = test_params.data_len;

    int64_t stamp;
    int64_t delta;
    uint32_t prog = 0;
    int err;

    err = test_init(conn_param, phy, data_len, BT_TEST_TYPE_SIMPLE);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }

    /* get cycle stamp */
    LOG_INF("Simple test started...");
    stamp = k_uptime_get_32();
    uint16_t buffer_size = test_data_buffer_size;
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
        prog += buffer_size;
    }

    delta = k_uptime_delta(&stamp);

    LOG_INF("Done");
    LOG_INF("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps", prog, prog / 1024, delta, ((uint64_t)prog * 8 / delta));

    /* read back char from peer */
    err = bt_performance_test_read(&performance_test);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }

    k_sem_take(&cmd_sync_sem, PERF_TEST_CONFIG_TIMEOUT);

    instruction_print();

    return;
}

struct k_work test_run_simple;
int test_run_cmd(const struct shell *shell, size_t argc, char **argv)
{
    // return test_run(test_params.conn_param, test_params.phy, test_params.data_len);
    shell_print(shell, "=== Start simple tests img transfer ===");
    /* initialize work item for test */
    k_work_init(&test_run_simple, test_run);
    k_work_submit_to_queue(&main_work_q , &test_run_simple);
    return 0;
}
