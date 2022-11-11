#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>
#include <main.h>

#include <zephyr/logging/log.h>

#include <performance_test.h>
#include <app_utils.h>
#include "sim_file.h"

extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;
static uint8_t test_runs = 1;
static int64_t stamp;

LOG_MODULE_DECLARE(main);

#define SIM_VALUE_BYTE_SIZE 3

static uint8_t write_data_to_buffer(uint8_t* buffer, uint32_t* data) {
    #if SIM_VALUE_BYTE_SIZE >= 1
                *(buffer + 0) = (uint8_t)(*(data));
    #endif
    #if SIM_VALUE_BYTE_SIZE >= 2
                *(buffer + 1) = (uint8_t)(*(data) >> 8);
    #endif
    #if SIM_VALUE_BYTE_SIZE >= 3
                *(buffer + 2) = (uint8_t)(*(data) >> 16);
    #endif
    #if SIM_VALUE_BYTE_SIZE >= 4
                *(buffer + 3) = (uint8_t)((*data) >> 24);
    #endif

    return SIM_VALUE_BYTE_SIZE;
}

static uint32_t send_test_sim_data()
{
    uint32_t sim_prog = 0;
    bool sim_skip_timestamp = false; // becasue there are times when timestamp is added as a last one
                                    // loop finishes cycle and then it's added again
                                    // this ruins a code structure so we want protect it whitout big changes
    uint16_t buffer_size = 0;
    uint32_t sim_written = 0;
    int err = 0;

    // treat data as one block of memory thanks to [] operator
    uint32_t *sim_ptr = ((uint32_t *)sim_data);

    while (sim_prog < SIM_SIZE)
    {
        /* 
         * if we are in middle of a buffer
         * set max buffer value as input
         * and update index of sim data
         */
        if ((SIM_SIZE - sim_prog) * SIM_VALUE_BYTE_SIZE > test_data_buffer_size)
        {
            buffer_size = test_data_buffer_size;
            buffer_size -= buffer_size % SIM_VALUE_BYTE_SIZE;
        }

        /* otherwise use only remaining sim data values */
        else
        {
            buffer_size = (SIM_SIZE - sim_prog) * 3;
        }


        for (int i = 0; i < buffer_size; i = i + SIM_VALUE_BYTE_SIZE, sim_prog++)
        {
            if (!(sim_prog % SIM_Y) && !sim_skip_timestamp) {
                /* add timestamp before every record */
                uint32_t data_stamp = k_uptime_get_32() - stamp;
                sim_written += write_data_to_buffer(test_data_buffer + i, &data_stamp);

                /* record new value in buffer */
                i = i + SIM_VALUE_BYTE_SIZE;

                /* check if we still in buffer */
                if (!(i < buffer_size)) {
                    sim_skip_timestamp = true;
                    // sim_prog will not be bumped here
                    break;
                }
            }
            sim_skip_timestamp = false;
            sim_written += write_data_to_buffer(test_data_buffer + i, sim_ptr + sim_prog);
        }

        err = bt_performance_test_write(&performance_test, test_data_buffer, buffer_size);
        if (err)
        {
            LOG_ERR("GATT write failed (err %d)", err);
            break;
        }
    }
    return sim_written;
}

static void test_run(struct k_work *item)
{
    const struct bt_le_conn_param *conn_param = test_params.conn_param;
    const struct bt_conn_le_phy_param *phy = test_params.phy;
    const struct bt_conn_le_data_len_param *data_len = test_params.data_len;

    int64_t delta;
    uint32_t prog = 0;

    int err;
    err = test_init(conn_param, phy, data_len, BT_TEST_TYPE_SIM);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }

    if (!ads129x_get_status()) {
        LOG_ERR("Adc data is not enabled");
    }

    /* get cycle stamp */
    LOG_INF("=== Start sim data transfer ===");
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
        shell_error(shell, "%s: This command require value in bytes, which tells how many bytes need to be send", argv[0]);
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

    /* initialize work item for test */
    k_work_init(&test_run_sim, test_run);
    k_work_submit_to_queue(&main_work_q, &test_run_sim);
    return 0;
}
