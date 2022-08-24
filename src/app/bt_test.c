#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <bluetooth/scan.h>
#include <bluetooth/gatt_dm.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <performance_test.h>
#include <bt_test.h>
#include <cmd.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(main);

K_SEM_DEFINE(performance_test_sem, 0, 1);

static volatile bool test_ready;
struct bt_conn *default_conn;
static struct bt_uuid *uuid128 = BT_UUID_PERF_TEST;
static struct bt_gatt_exchange_params exchange_params;
static struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);

struct bt_performance_test performance_test;
extern const struct bt_performance_test_cb performance_test_cb;

struct bt_conn *getSettings(void)
{
    return default_conn;
}

volatile bool isTestReady(void)
{
    return test_ready;
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xBB, 0x4A, 0xFF, 0x4F, 0xAD, 0x03, 0x41, 0x5D,
                  0xA9, 0x6C, 0x9D, 0x6C, 0xDD, 0xDA, 0x83, 0x04),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const char *phy2str(uint8_t phy)
{
    switch (phy)
    {
    case 0:
        return "No packets";
    case BT_GAP_LE_PHY_1M:
        return "LE 1M";
    case BT_GAP_LE_PHY_2M:
        return "LE 2M";
    case BT_GAP_LE_PHY_CODED:
        return "LE Coded";
    default:
        return "Unknown";
    }
}

// ##### SCAN (Master)

void scan_filter_match(struct bt_scan_device_info *device_info,
                       struct bt_scan_filter_match *filter_match,
                       bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable);
}

void scan_filter_no_match(struct bt_scan_device_info *device_info,
                          bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    LOG_INF("Filter not match. Address: %s connectable: %d", addr, connectable);
}

void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    LOG_INF("Connecting failed");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match,
                scan_connecting_error, NULL);

static void scan_init(void)
{
    int err;
    struct bt_le_scan_param scan_param = {
        .type = BT_LE_SCAN_TYPE_PASSIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = 0x0010,
        .window = 0x0010,
    };

    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
        .scan_param = &scan_param,
        .conn_param = conn_param};

    bt_scan_init(&scan_init);
    bt_scan_cb_register(&scan_cb);

    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, uuid128);
    if (err)
    {
        LOG_ERR("Scanning filters cannot be set");
        return;
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err)
    {
        LOG_ERR("Filters cannot be turned on");
    }

    LOG_INF("Scan filters are enabled");
}

void scan_start(struct k_work *item)
{
    // think of bt_le_scan_start
    int err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
    if (err)
    {
        LOG_ERR("Starting scanning failed (err %d)", err);
    }
    else
    {
        LOG_INF("Scanning started");
    }
    return;
}

void adv_start(struct k_work *item)
{
    struct bt_le_adv_param *adv_param =
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME,
                        BT_GAP_ADV_FAST_INT_MIN_2,
                        BT_GAP_ADV_FAST_INT_MAX_2,
                        NULL);
    int err;

    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err)
    {
        LOG_ERR("Failed to start advertiser (%d)", err);
    }
    else
    {
        LOG_ERR("Start advertiser (%d)", ad->type);
    }
    return;
}

/**
 * @brief Determine maximum transmission unit (MTU) for a connection.
 *
 * @param conn - handle for the connection
 * @param att_err - error code for the ATT operation
 * @param params - parameters for the ATT operation
 *
 * @details This function is called by master to init the MTU exchange.
 *          The function will determine the maximum transmission unit (MTU).
 *          This procedure is used only whenever either the client or the server (or both)
 *          can handle MTUs longer than the default ATT_MTU of 23 bytes (see “Logical Link Control and Adaptation Protocol (L2CAP)”)
 *          and wants to inform the other end that it can send packets longer than the default values that the specification requires.
 *          L2CAP will then fragment these bigger packets into small Link Layers packets and recombine them from small Link Layers packets.
 *
 * @return * determine
 */
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    struct bt_conn_info info = {0};
    int err;

    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");

    err = bt_conn_get_info(conn, &info);
    if (err)
    {
        LOG_ERR("Failed to get connection info %d", err);
        return;
    }

    if (info.role == BT_CONN_ROLE_CENTRAL)
    {
        instruction_print();
        test_ready = true;
    }
}

// scan callbacks
static void discovery_complete(struct bt_gatt_dm *dm,
                               void *context)
{
    int err;
    struct bt_performance_test *performance_test = context;

    LOG_INF("Service discovery completed");

    bt_gatt_dm_data_print(dm);
    bt_performance_test_handles_assign(dm, performance_test);
    bt_gatt_dm_data_release(dm);
}

static void discovery_service_not_found(struct bt_conn *conn,
                                        void *context)
{
    LOG_WRN("Service not found");
}

static void discovery_error(struct bt_conn *conn,
                            int err,
                            void *context)
{
    LOG_ERR("Error while discovering GATT database: (%d)", err);
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_complete,
    .service_not_found = discovery_service_not_found,
    .error_found = discovery_error,
};

// ##### End

static void connected(struct bt_conn *conn, uint8_t hci_err)
{
    struct bt_conn_info info = {0};
    int err;

    if (hci_err)
    {
        if (hci_err == BT_HCI_ERR_UNKNOWN_CONN_ID)
        {
            /* Canceled creating connection */
            return;
        }

        LOG_ERR("Connection failed (err 0x%02x)", hci_err);
        return;
    }

    if (default_conn)
    {
        LOG_WRN("Connection exists, disconnect second connection");
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    default_conn = bt_conn_ref(conn);

    err = bt_conn_get_info(default_conn, &info);
    if (err)
    {
        LOG_ERR("Failed to get connection info %d", err);
        return;
    }

    LOG_INF("Connected as %s", info.role == BT_CONN_ROLE_CENTRAL ? "central" : "peripheral");
    LOG_INF("Conn. interval is %u units (%u ms)", info.le.interval, (uint32_t)(info.le.interval * UNIT_SCALER));

    if (info.role == BT_CONN_ROLE_CENTRAL)
    {
        err = bt_gatt_dm_start(default_conn,
                               BT_UUID_PERF_TEST,
                               &discovery_cb,
                               &performance_test);

        if (err)
        {
            LOG_ERR("Discover failed (err %d)", err);
        }
    }

    exchange_params.func = exchange_func;
    err = bt_gatt_exchange_mtu(default_conn, &exchange_params);
    if (err)
    {
        LOG_INF("MTU exchange failed (err %d)", err);
    }
    else
    {
        LOG_INF("MTU exchange pending");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    struct bt_conn_info info = {0};
    int err;

    LOG_INF("Disconnected (reason 0x%02x)", reason);

    test_ready = false;
    if (default_conn)
    {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    err = bt_conn_get_info(conn, &info);
    if (err)
    {
        LOG_ERR("Failed to get connection info (%d)", err);
        return;
    }
}

void restore_state(void)
{
    test_ready = false;
    if (default_conn)
    {
        bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    return;
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
    LOG_INF("Connection parameters update request received.");
    LOG_INF("Minimum interval: %d (%d ms), Maximum interval: %d (%d ms)",
            param->interval_min,
            (uint32_t)(param->interval_min * UNIT_SCALER),
            param->interval_max,
            (uint32_t)(param->interval_max * UNIT_SCALER));
    LOG_INF("Latency: %d, Timeout: %d", param->latency, param->timeout);

    return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout)
{
    LOG_INF("Connection parameters updated");
    LOG_INF("Interval: %d (%d ms), latency: %d, timeout: %d",
            interval, (uint32_t)(interval * UNIT_SCALER), latency, timeout);

    k_sem_give(&performance_test_sem);
}

static void le_phy_updated(struct bt_conn *conn,
                           struct bt_conn_le_phy_info *param)
{
    LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s", phy2str(param->tx_phy), phy2str(param->rx_phy));

    k_sem_give(&performance_test_sem);
}

static void le_data_length_updated(struct bt_conn *conn,
                                   struct bt_conn_le_data_len_info *info)
{
    LOG_INF("LE data len updated: TX (len: %d time: %d) RX (len: %d time: %d)",
            info->tx_max_len,
            info->tx_max_time,
            info->rx_max_len,
            info->rx_max_time);

    k_sem_give(&performance_test_sem);
}

K_THREAD_STACK_DEFINE(bt_conf_thread_area, 512);
void connection_config_update(void *phy, void *data_len, void *conn_param)
{

    int err = 0;
    LOG_INF("PHY update pending");
    err = bt_conn_le_phy_update(default_conn, phy);
    if (err)
    {
        LOG_ERR("PHY update failed: %d", err);
        return;
    }

    LOG_INF("LE Data length update pending");
    err = bt_conn_le_data_len_update(default_conn, data_len);
    if (err)
    {
        LOG_ERR("LE data length update failed: %d", err);
        return;
    }

    LOG_INF("Connection parameters update pending");
    err = bt_conn_le_param_update(default_conn, conn_param);
    if (err)
    {
        LOG_ERR("Connection parameters update failed: %d", err);
        return;
    }
}

int connection_configuration_set(const struct shell *shell,
                                 const struct bt_le_conn_param *conn_param,
                                 const struct bt_conn_le_phy_param *phy,
                                 const struct bt_conn_le_data_len_param *data_len)
{
    int err;
    struct bt_conn_info info = {0};

    err = bt_conn_get_info(default_conn, &info);
    if (err)
    {
        LOG_ERR("Failed to get connection info %d", err);
        return err;
    }

    if (info.role != BT_CONN_ROLE_CENTRAL)
    {
        LOG_ERR("'run' command shall be executed only on the master board");
    }

    /* to fix race conditions spawn thread which will */
    struct k_thread bt_conf_thread;
    k_thread_create(&bt_conf_thread, bt_conf_thread_area,
                    K_THREAD_STACK_SIZEOF(bt_conf_thread_area),
                    connection_config_update,
                    (void *)phy, (void *)data_len, (void *)conn_param,
                    7, 0, K_NO_WAIT);

    err = k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);
    if (err)
    {
        LOG_ERR("PHY update timeout");
        k_thread_join(&bt_conf_thread, K_MSEC(100));
        return err;
    }

    err = k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);
    if (err)
    {
        LOG_ERR("LE Data Length update timeout");
        k_thread_join(&bt_conf_thread, K_MSEC(100));
        return err;
    }

    err = k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);
    if (err)
    {
        LOG_ERR("Connection parameters update timeout");
        k_thread_join(&bt_conf_thread, K_MSEC(100));
        return err;
    }

    LOG_INF("Connection parameters updated");
    return 0;
}

void bt_init(void)
{
    int err = -1;

    static struct bt_conn_cb conn_callbacks = {
        .connected = connected,
        .disconnected = disconnected,
        .le_param_req = le_param_req,
        .le_param_updated = le_param_updated,
        .le_phy_updated = le_phy_updated,
        .le_data_len_updated = le_data_length_updated};

    LOG_INF("Starting Bluetooth Performance test");

    err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    bt_conn_cb_register(&conn_callbacks);
    LOG_INF("Bluetooth initialized");

    scan_init();

    err = bt_performance_test_init(&performance_test, &performance_test_cb);
    if (err)
    {
        LOG_ERR("Performance test service initialization failed.");
        return;
    }
}