#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>
#include <main.h>

#include <zephyr/logging/log.h>

#include <performance_test.h>
#include <app_utils.h>
#include "sim_file.h"
#include <cmd.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_DECLARE(main);

K_THREAD_STACK_DEFINE(sim_stack, 4096);
static struct k_thread sim_thread;

RING_BUF_DECLARE(sim_ring_buffer, 247*4);
static uint32_t bytes_to_send = 0;

int32_t sim_load_row_data(uint8_t *load_data, uint32_t* sim_pos) {
    uint8_t* tmp = utils_write_timestamp(load_data);

    *sim_pos = (*sim_pos) % SIM_X;
    for (int i = 0; i < SIM_Y; i++)
    {
        tmp = conv_u24_to_raw(sim_data[*sim_pos][i], tmp, 0);
    }
    (*sim_pos)++;
    return SIM_Y*3 + 3;
}


int32_t sim_get_data(uint8_t *load_data, int32_t size, uint32_t* sim_pos)
{
    int32_t loaded = 0;
    uint32_t buffer_size = size;

    //* determine number of data
    if (size > test_params.data_len->tx_max_len - 7)
    {
        buffer_size = test_params.data_len->tx_max_len - 7;
    }

    while (loaded < buffer_size)
    {
        loaded += sim_load_row_data(load_data, sim_pos);
    }

    return loaded;
}

static uint32_t send_test_sim_data(uint32_t _bytes_to_send)
{
    // data tracing
    uint32_t sim_pos = 0;
    uint32_t load_data = 0;
    uint32_t remaining = 0;

    // ring buff
    uint8_t* tmp_buff;
    uint32_t claimed = 0;

    // ble monitoring
    uint32_t sent_data = 0;
    int err = 0;

    while (load_data < _bytes_to_send)
    {
        remaining = _bytes_to_send - load_data;

        ring_buf_put_claim(&sim_ring_buffer, &tmp_buff, remaining);
        claimed = sim_get_data(tmp_buff, remaining, &sim_pos);
        ring_buf_put_finish(&sim_ring_buffer, claimed);

        sent_data = claimed >= test_params.data_len->tx_max_len - 7 ? test_params.data_len->tx_max_len - 7 : claimed;
        claimed = ring_buf_get_claim(&sim_ring_buffer, &tmp_buff, sent_data);
        err = bt_performance_test_write(&performance_test, tmp_buff, claimed);
        if (err)
        {
            LOG_ERR("GATT write failed (err %d)", err);
            break;
        }
        ring_buf_get_finish(&sim_ring_buffer, claimed);
        load_data += claimed;
    }
    return load_data;
}

static void sim_test_run()
{
    const struct bt_le_conn_param *conn_param = test_params.conn_param;
    const struct bt_conn_le_phy_param *phy = test_params.phy;
    const struct bt_conn_le_data_len_param *data_len = test_params.data_len;

    int64_t delta;
    int64_t stamp;

    int err;
    err = test_init(conn_param, phy, data_len, BT_TEST_TYPE_SIM);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }

    /* get cycle stamp */
    LOG_INF("=== Start sim data transfer ===");
    stamp = k_uptime_get_32();
    bytes_to_send = send_test_sim_data(bytes_to_send);
    delta = k_uptime_delta(&stamp);
    LOG_INF("Done");
    LOG_INF("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps", bytes_to_send, bytes_to_send / 1024, delta, ((uint64_t)bytes_to_send * 8 / delta));

    /* read back char from peer */
    err = bt_performance_test_read(&performance_test);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }
    k_sem_take(&cmd_sync_sem, PERF_TEST_CONFIG_TIMEOUT);
    cmd_bt_dump_data(NULL, 0);

    instruction_print();
    return;
}

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

    bytes_to_send = strtol(argv[1], NULL, 10);
    if (bytes_to_send < 100)
    {
        shell_error(shell, "Invalid parameter %" PRIu8, bytes_to_send);
        return -EINVAL;
    }

    /* initialize work item for test */
    k_tid_t my_tid = k_thread_create(&sim_thread, sim_stack,
                                    K_THREAD_STACK_SIZEOF(sim_stack),
                                    sim_test_run,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(my_tid, "cmd_sim");

    return 0;
}
