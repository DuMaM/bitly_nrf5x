#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

#include <performance_test.h>
#include <cmd.h>
#include <cmd_run.h>
#include <bt_test.h>

LOG_MODULE_DECLARE(main);
K_SEM_DEFINE(cmd_sync_sem, 0, 1);

/* a test_data_buffer data buffer */
uint8_t test_data_buffer[502];
const uint16_t test_data_buffer_size = sizeof(test_data_buffer) / sizeof(test_data_buffer[0]);

static void get_rssi_power(struct bt_conn *conn)
{
    uint16_t conn_handle;

    int err = bt_hci_get_conn_handle(conn, &conn_handle);

    if (err)
    {
        LOG_ERR("No connection handle (err %d)", err);
        return;
    }

    struct bt_hci_cp_read_rssi *cp;
    struct bt_hci_rp_read_rssi *rp;
    struct net_buf *buf;
    struct net_buf *rsp = NULL;

    buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
    if (!buf)
    {
        LOG_ERR("Cannot allocate buffer to get RSSI power");
        return;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(conn_handle);

    err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
    if (err)
    {
        LOG_ERR("Get rssi power value (err: %d)", err);
    }
    else
    {
        rp = (struct bt_hci_rp_read_rssi *)(rsp->data);
        LOG_DBG("RSSI power returned by command: %" PRId8, rp->rssi);
    }

    if (rsp)
    {
        net_buf_unref(rsp);
    }
}

static void get_tx_power(struct bt_conn *conn)
{
    struct bt_conn_le_tx_power tx_power = {.phy = 0};
    int error = bt_conn_le_get_tx_power_level(conn, &tx_power);
    if (!error)
    {
        LOG_INF("TX power returned by command: %" PRId8, tx_power.current_level);
    }
}

static uint8_t performance_test_read(const bt_performance_test_metrics_t *met)
{
    LOG_INF("[peer] received %u bytes (%u KB) in %u GATT writes at %u kbps", met->write_len, met->write_len / 1024, met->write_count, met->write_rate / 1000);
    k_sem_give(&cmd_sync_sem);

    return BT_GATT_ITER_STOP;
}

/**
 * @brief Executed every time new data was received from master
 *
 * @param met Prt to current metrics values
 */
static void performance_test_received(const bt_performance_test_metrics_t *met)
{
    static uint32_t kb = 0;
    static uint16_t kb_line = 0;

    /* init case for new package sent */
    if (met->write_len == 0)
    {
        kb = 0;
        kb_line = 0;
        LOG_INF("Test started");
    }

    /* show progress of data transfer */
    uint32_t kb_tmp = met->write_len / 10240;
    if (kb_tmp != kb)
    {
        kb = kb_tmp;
        LOG_INF("<- 10kB package (%"PRIu32")...", met->write_len);
    }

    // /*
    //  * classic terminal have line width 80 characters long
    //  * so we want to add formatting so it will be easier to read
    //  */
    // uint16_t kb_line_tmp = kb / 64;
    // if (kb_line != kb_line_tmp)
    // {
    //     get_rssi_power(default_conn);
    // }
}

/**
 * @brief Executed when data is sended back to master
 *
 * @param met Ptr to structure with metrics values
 */
static void performance_test_send(const bt_performance_test_metrics_t *met)
{
    LOG_INF("[local] received %u bytes (%u KB) in %u GATT writes at %u kbps", met->write_len, met->write_len / 1024, met->write_count, met->write_rate / 1000);
}

struct bt_performance_test_cb performance_test_cb = {
    .data_read = performance_test_read,
    .data_received = performance_test_received,
    .data_send = performance_test_send,
    .package_sent = NULL
};

int test_init(const struct bt_le_conn_param *conn_param,
              const struct bt_conn_le_phy_param *phy,
              const struct bt_conn_le_data_len_param *data_len,
              const bt_test_type_t type)
{
    int err;

    if (!getSettings())
    {
        LOG_ERR("Device is disconnected. Connect to the peer device before running test");
        return -EFAULT;
    }

    if (!isTestReady())
    {
        LOG_ERR("Device is not ready. Please wait for the service discovery and MTU exchange end");
        return 0;
    }

    err = connection_configuration_set();
    if (err)
    {
        LOG_ERR("Connection settings was not set correctly");
        return err;
    }

    /* reset peer metrics */
    err = bt_performance_test_set_type(&performance_test, type);
    if (err)
    {
        LOG_ERR("Reset peer metrics failed.");
        return err;
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_bt_test,
                               SHELL_CMD(ber_alt, NULL, "Tests ber signal with pattern |11|00|11|00", test_run_ber_alternating_cmd),
                               SHELL_CMD(ber_const, NULL, "Tests ber signal with pattern |10|10|10|10", test_run_ber_oppsed_cmd),
                               SHELL_CMD(sim, NULL, "Tests with simulated ECG signal", sim_run_cmd),
                               SHELL_CMD(analog, NULL, "Tests with ECG signal", adc_run_cmd),
                               SHELL_CMD(test_picture, NULL, "Tests transition with example picture", test_run_cmd),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(run, &sub_bt_test, "Run the test", default_cmd);
