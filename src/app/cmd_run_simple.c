#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>
#include <main.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include <performance_test.h>
#include <bt_test.h>
#include "img_file.h"

extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;

LOG_MODULE_DECLARE(main);


K_THREAD_STACK_DEFINE(simple_stack, 4096);
struct k_thread simple_thread = {.name = "cmd_simple"};

static void simple_test_run()
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
    uint32_t target_frame_size = test_params.data_len->tx_max_len - 7;
    /* get cycle stamp */
    LOG_INF("=== Start simple tests img transfer ===");
    stamp = k_uptime_get_32();
    uint16_t buffer_size = target_frame_size;
    uint8_t *img_prt = NULL;


    while (prog < IMG_SIZE)
    {
        img_prt = ((uint8_t *)img) + prog;
        buffer_size = IMG_SIZE - prog;
        if (IMG_SIZE - prog > target_frame_size)
        {
            buffer_size = target_frame_size;
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


    /* read back char from peer */
    err = bt_performance_test_read(&performance_test);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }

    k_sem_take(&cmd_sync_sem, PERF_TEST_CONFIG_TIMEOUT);
    LOG_INF("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps", prog, prog / 1024, delta, ((uint64_t)prog * 8 / delta));

    instruction_print();

    return;
}

int test_run_cmd(const struct shell *shell, size_t argc, char **argv)
{
    /* initialize work item for test */
    /* initialize work item for test */
    k_thread_create(&simple_thread, simple_stack,
                                    K_THREAD_STACK_SIZEOF(simple_stack),
                                    simple_test_run,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);
    return 0;
}
