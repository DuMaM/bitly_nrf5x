#include <cmd_run.h>
#include <cmd.h>

#include <bt_test.h>
#include <main.h>

#include <zephyr/logging/log.h>

#include <performance_test.h>
#include <app_utils.h>
#include <spi_adc.h>

extern uint8_t test_data_buffer[];
extern uint16_t test_data_buffer_size;
static uint8_t test_runs = 1;
static int64_t stamp;

LOG_MODULE_DECLARE(main);

#ifdef CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP
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

static uint32_t bytes_to_send = 1024;

static uint32_t send_test_sim_data(uint32_t _bytes_to_send)
{
    uint32_t prog = 0;
    uint8_t *analog_data_ptr = NULL;
    uint16_t analog_data_size = (test_params.data_len->tx_max_len / ADS129x_DATA_BUFFER_SIZE) * ADS129x_DATA_BUFFER_SIZE ;
    int err = 0;

    while (prog < _bytes_to_send)
    {
        /**
         * TODO: add scaling from shell
         * It would be good to send optimal number of data
         * during each transfer
         * also when we change a number o leads
         */
        if (!ads129x_get_data(analog_data_ptr, analog_data_size))
        {
            continue;
        }

        err = bt_performance_test_write(&performance_test, analog_data_ptr, analog_data_size);
        if (err)
        {
            LOG_ERR("GATT write failed (err %d)", err);
            break;
        }

        prog += analog_data_size;
    }
    return prog;
}

static void test_run(struct k_work *item)
{
    const struct bt_le_conn_param *conn_param = test_params.conn_param;
    const struct bt_conn_le_phy_param *phy = test_params.phy;
    const struct bt_conn_le_data_len_param *data_len = test_params.data_len;

    int64_t delta;
    uint32_t prog = 0;

    int err;
    err = test_init(conn_param, phy, data_len, BT_TEST_TYPE_ANALOG);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }

    /* get cycle stamp */
    LOG_INF("=== Start analog data transfer ===");
    ads129x_reset_data();
    stamp = k_uptime_get_32();
    prog = send_test_sim_data(bytes_to_send);
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

struct k_work test_run_adc;
int adc_run_cmd(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 1)
    {
        shell_help(shell);
        LOG_ERR("%s: This command require value in bytes, which tells how many bytes need to be send", argv[0]);
        return SHELL_CMD_HELP_PRINTED;
    }

    if (argc > 2)
    {
        LOG_ERR("%s: bad parameters count", argv[0]);
        return -EINVAL;
    }

    bytes_to_send = strtoul(argv[1], NULL, 10);
    if (ADS129x_DATA_BUFFER_SIZE > bytes_to_send)
    {
        LOG_ERR("Invalid parameter %" PRIu8 ", it should be bigger then max data send from spi device", bytes_to_send);
        return -EINVAL;
    }

    /* initialize work item for test */
    k_work_init(&test_run_adc, test_run);
    k_work_submit_to_queue(&main_work_q, &test_run_adc);
    return 0;
}

#else

int adc_run_cmd(const struct shell *shell, size_t argc, char **argv)
{
    shell_help(shell);
    LOG_ERR("%s: Command is not supported by this platform", argv[0]);
    return SHELL_CMD_HELP_PRINTED;
}

#endif