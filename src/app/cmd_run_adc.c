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

LOG_MODULE_DECLARE(main);

#ifdef CONFIG_SPI

K_THREAD_STACK_DEFINE(ecg_ble_stack, 4096);
struct k_sem cmd_count_writes_num;

struct k_thread ecg_ble_thread;
static uint32_t bytes_to_send = 1024;

static uint32_t send_test_ecg_data(uint32_t _bytes_to_send)
{
    uint32_t prog = 0;
    uint8_t *analog_data_ptr = test_data_buffer;
    uint32_t analog_data_size = 0;
    int err = 0;

    /*
     * we always waiting for data to match whole buffer
     * so data requested should also match it
     * otherwise we should ignore it
     */
    bytes_to_send = set_bytes_to_send(_bytes_to_send);
    LOG_INF("Sending %"PRIu32" bytes (value after rounding to max packet size)", bytes_to_send);

    uint32_t target_frame_size = test_params.data_len->tx_max_len - 7;
    if (test_params.fit_buffer) {
        target_frame_size =  ((test_params.data_len->tx_max_len - 7) / ADS129x_DATA_BUFFER_SIZE) * ADS129x_DATA_BUFFER_SIZE;
    }

    while (prog < bytes_to_send)
    {
        analog_data_size = bytes_to_send - prog;
        if (target_frame_size <= analog_data_size) {
            analog_data_size = target_frame_size;
        }

        /*
         * TODO: add scaling from shell
         * It would be good to send optimal number of data
         * during each transfer
         * also when we change a number o leads
         */
        if (ads129x_get_data(analog_data_ptr, analog_data_size) <= 0)
        {
            /*
             * sleep for one connection interval
             * this will allow to buffer spi data
             * after that we have constant data stream which
             * can be send during connection event
             *
             * I added a weight in 0.9 value to start a bit earlier
             * data feeding for connection event.
             */
            // k_msleep(test_params.conn_param->interval_max * UNIT_SCALER * 0.5);
            continue;
        }


        //if (!k_sem_take(&cmd_count_writes_num, K_FOREVER)) {
            err = bt_performance_test_write(&performance_test, analog_data_ptr, analog_data_size);
            if (err)
            {
                LOG_ERR("GATT write failed (err %d)", err);
                break;
            }
            prog += analog_data_size;

        //}
    }
    return prog;
}

void notify_packet_sent(struct bt_conn *conn, void *user_data) {
    k_sem_give(&cmd_count_writes_num);
}


static void adc_test_run()
{
    int64_t delta;
    int64_t stamp;
    uint32_t prog = 0;

    // add callback to monitor data sent and reset data
    extern struct bt_performance_test_cb performance_test_cb;
    //performance_test_cb.package_sent = &notify_packet_sent;
    k_sem_init(&cmd_count_writes_num, 2, 2);

    // start transmission
    stamp = k_uptime_get_32();
    prog = send_test_ecg_data(bytes_to_send);
    delta = k_uptime_delta(&stamp);

    /* read back char from peer */
    int err = bt_performance_test_read(&performance_test);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return;
    }

    k_sem_take(&cmd_sync_sem, PERF_TEST_CONFIG_TIMEOUT);

    cmd_bt_dump_data(NULL, 0);
    LOG_INF("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps", prog, prog / 1024, delta, ((uint64_t)prog * 8 / delta));

    // clean after tests
    ads129x_data_disable();
    performance_test_cb.package_sent = NULL;

    //instruction_print();

    return;
}

int adc_run_cmd(const struct shell *shell, size_t argc, char **argv)
{
    const struct bt_le_conn_param *conn_param = test_params.conn_param;
    const struct bt_conn_le_phy_param *phy = test_params.phy;
    const struct bt_conn_le_data_len_param *data_len = test_params.data_len;

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

    bytes_to_send = strtoul(argv[1], NULL, 10);
    if (ADS129x_DATA_BUFFER_SIZE > bytes_to_send && (bytes_to_send < BER_MIN_DATA && cmd_status_logs()))
    {
        shell_error(shell, "Invalid parameter %" PRIu32 ", it should be bigger then max data send from spi device", bytes_to_send);
        return -EINVAL;
    }

    LOG_DBG("Data read speed: %"PRIu32, bytes_to_send);

    // allow to return from shell
    /* get cycle stamp */
    LOG_INF("=== Reseting data buffer ===");
    ads129x_data_enable();
    LOG_INF("=== Start analog data transfer ===");
    k_usleep(500);

    int err = test_init(conn_param, phy, data_len, BT_TEST_TYPE_ANALOG);
    if (err)
    {
        LOG_ERR("GATT read failed (err %d)", err);
        return 0;
    }

    /* initialize work item for test */
    k_tid_t my_tid = k_thread_create(&ecg_ble_thread, ecg_ble_stack,
                                    K_THREAD_STACK_SIZEOF(ecg_ble_stack),
                                    adc_test_run,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    k_thread_name_set(my_tid, "cmd_adc");

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