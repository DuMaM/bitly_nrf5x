#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>
#include <main.h>

#include <zephyr/logging/log.h>

#include <performance_test.h>
#include "sim_file.h"

extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;
static uint8_t test_runs = 1;

LOG_MODULE_DECLARE(main);

#define SIM_VALUE_BYTE_SIZE 3
static uint32_t send_test_sim_data()
{
    uint32_t prog = 0;
    uint16_t buffer_size = 0;
    uint32_t *sim_ptr = NULL;
    int err = 0;

    while (prog < SIM_SIZE)
    {
        sim_ptr = ((uint32_t *)sim_data) + prog;
        if ((SIM_SIZE - prog) * SIM_VALUE_BYTE_SIZE > test_data_buffer_size)
        {
            buffer_size = test_data_buffer_size;
            buffer_size -= buffer_size % SIM_VALUE_BYTE_SIZE;
            prog += buffer_size / SIM_VALUE_BYTE_SIZE;
        }
        else
        {
            buffer_size = (SIM_SIZE - prog) * 3;
            prog += SIM_SIZE - prog;
        }

        for (int i = 0, j = 0; i < buffer_size; i = i + SIM_VALUE_BYTE_SIZE, j++)
        {
#if SIM_VALUE_BYTE_SIZE >= 1
            test_data_buffer[i] = sim_ptr[j];
#endif
#if SIM_VALUE_BYTE_SIZE >= 2
            test_data_buffer[i + 1] = sim_ptr[j] >> 8;
#endif
#if SIM_VALUE_BYTE_SIZE >= 3
            test_data_buffer[i + 2] = sim_ptr[j] >> 16;
#endif
#if SIM_VALUE_BYTE_SIZE >= 4
            test_data_buffer[i + 3] = sim_ptr[j] >> 24;
#endif
        }

        err = bt_performance_test_write(&performance_test, test_data_buffer, buffer_size);
        if (err)
        {
            LOG_ERR("GATT write failed (err %d)", err);
            break;
        }
    }
    return SIM_VALUE_BYTE_SIZE * prog;
}

static void test_run(struct k_work *item)
{
    const struct bt_le_conn_param *conn_param = test_params.conn_param;
    const struct bt_conn_le_phy_param *phy = test_params.phy;
    const struct bt_conn_le_data_len_param *data_len = test_params.data_len;

    int64_t stamp;
    int64_t delta;
    uint32_t prog = 0;

    int err;
    err = test_init(conn_param, phy, data_len, BT_TEST_TYPE_SIM);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }

    /* get cycle stamp */
    LOG_INF("Sim test started...");
    stamp = k_uptime_get_32();
    for (int i = 0; i < test_runs; i++)
    {
        prog += send_test_sim_data();
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

struct k_work test_run_sim;
int sim_run_cmd(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 1)
    {
        shell_help(shell);
        return SHELL_CMD_HELP_PRINTED;
    }

    if (argc > 2)
    {
        shell_error(shell, "%s: bad parameters count", argv[0]);
        return -EINVAL;
    }

    test_runs = strtol(argv[1], NULL, 10);
    if (test_runs < 1)
    {
        shell_error(shell, "Invalid parameter %" PRIu8, test_runs);
        return -EINVAL;
    }

    shell_print(shell, "=== Start sim data transfer ===");

    /* initialize work item for test */
    k_work_init(&test_run_sim, test_run);
    k_work_submit_to_queue(&main_work_q, &test_run_sim);
    return 0;
}
