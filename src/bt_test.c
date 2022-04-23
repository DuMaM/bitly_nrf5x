#include <bt_test.h>
#include <zephyr/types.h>
#include <cmds.h>
#include <sys/byteorder.h>

K_SEM_DEFINE(performance_test_sem, 0, 1);

static volatile bool data_length_req;
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

// ##### SCAN

void scan_filter_match(struct bt_scan_device_info *device_info,
                       struct bt_scan_filter_match *filter_match,
                       bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    printk("Filters matched. Address: %s connectable: %d\n",
           addr, connectable);
}

void scan_filter_no_match(struct bt_scan_device_info *device_info,
                          bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr));

    printk("Filter not match. Address: %s connectable: %d\n",
           addr, connectable);
}

void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    printk("Connecting failed\n");
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, scan_filter_no_match,
                scan_connecting_error, NULL);

// ##### End

static void exchange_func(struct bt_conn *conn, uint8_t att_err,
                          struct bt_gatt_exchange_params *params)
{
    struct bt_conn_info info = {0};
    int err;

    printk("MTU exchange %s\n", att_err == 0 ? "successful" : "failed");

    err = bt_conn_get_info(conn, &info);
    if (err)
    {
        printk("Failed to get connection info %d\n", err);
        return;
    }

    if (info.role == BT_CONN_ROLE_CENTRAL)
    {
        instruction_print();
        test_ready = true;
    }
}

static void discovery_complete(struct bt_gatt_dm *dm,
                               void *context)
{
    int err;
    struct bt_performance_test *performance_test = context;

    printk("Service discovery completed\n");

    bt_gatt_dm_data_print(dm);
    bt_performance_test_handles_assign(dm, performance_test);
    bt_gatt_dm_data_release(dm);

    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(default_conn, &exchange_params);
    if (err)
    {
        printk("MTU exchange failed (err %d)\n", err);
    }
    else
    {
        printk("MTU exchange pending\n");
    }
}

static void discovery_service_not_found(struct bt_conn *conn,
                                        void *context)
{
    printk("Service not found\n");
}

static void discovery_error(struct bt_conn *conn,
                            int err,
                            void *context)
{
    printk("Error while discovering GATT database: (%d)\n", err);
}

struct bt_gatt_dm_cb discovery_cb = {
    .completed = discovery_complete,
    .service_not_found = discovery_service_not_found,
    .error_found = discovery_error,
};

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

        printk("Connection failed (err 0x%02x)\n", hci_err);
        return;
    }

    if (default_conn)
    {
        printk("Connection exists, disconnect second connection\n");
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

    default_conn = bt_conn_ref(conn);

    err = bt_conn_get_info(default_conn, &info);
    if (err)
    {
        printk("Failed to get connection info %d\n", err);
        return;
    }

    printk("Connected as %s\n", info.role == BT_CONN_ROLE_CENTRAL ? "central" : "peripheral");
    printk("Conn. interval is %u units (%u ms)\n", info.le.interval, (uint32_t)(info.le.interval * UNIT_SCALER));

    if (info.role == BT_CONN_ROLE_CENTRAL)
    {
        err = bt_gatt_dm_start(default_conn,
                               BT_UUID_PERF_TEST,
                               &discovery_cb,
                               &performance_test);

        if (err)
        {
            printk("Discover failed (err %d)\n", err);
        }
    }
}

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
        printk("Scanning filters cannot be set\n");

        return;
    }

    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err)
    {
        printk("Filters cannot be turned on\n");
    }
}

void scan_start(void)
{
    // think of bt_le_scan_start
    int err = bt_scan_start(BT_SCAN_TYPE_SCAN_PASSIVE);
    if (err)
    {
        printk("Starting scanning failed (err %d)\n", err);
        return;
    }
}

void adv_start(void)
{
    struct bt_le_adv_param *adv_param =
        BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME,
                        BT_GAP_ADV_FAST_INT_MIN_2,
                        BT_GAP_ADV_FAST_INT_MAX_2,
                        NULL);
    int err;

    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd,
                          ARRAY_SIZE(sd));
    if (err)
    {
        printk("Failed to start advertiser (%d)\n", err);
        return;
    } else {
        printk("Start advertiser (%d)\n", ad->type);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    struct bt_conn_info info = {0};
    int err;

    printk("Disconnected (reason 0x%02x)\n", reason);

    test_ready = false;
    if (default_conn)
    {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }

    err = bt_conn_get_info(conn, &info);
    if (err)
    {
        printk("Failed to get connection info (%d)\n", err);
        return;
    }

    /* Re-connect using same roles */
    if (info.role == BT_CONN_ROLE_CENTRAL)
    {
        scan_start();
    }
    else
    {
        adv_start();
    }
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
    printk("Connection parameters update request received.\n");
    printk("Minimum interval: %d (%d ms), Maximum interval: %d (%d ms)\n", param->interval_min,
           (uint32_t)(param->interval_min * UNIT_SCALER),
           param->interval_max,
           (uint32_t)(param->interval_max * UNIT_SCALER));
    printk("Latency: %d, Timeout: %d\n", param->latency, param->timeout);

    return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                             uint16_t latency, uint16_t timeout)
{
    printk("Connection parameters updated.\n"
           " interval: %d (%d ms), latency: %d, timeout: %d\n",
           interval, (uint32_t)(interval * UNIT_SCALER), latency, timeout);

    k_sem_give(&performance_test_sem);
}

static void le_phy_updated(struct bt_conn *conn,
                           struct bt_conn_le_phy_info *param)
{
    printk("LE PHY updated: TX PHY %s, RX PHY %s\n", phy2str(param->tx_phy), phy2str(param->rx_phy));

    k_sem_give(&performance_test_sem);
}

static void le_data_length_updated(struct bt_conn *conn,
                                   struct bt_conn_le_data_len_info *info)
{
    if (!data_length_req)
    {
        return;
    }

    printk("LE data len updated: TX (len: %d time: %d)"
           " RX (len: %d time: %d)\n",
           info->tx_max_len,
           info->tx_max_time, info->rx_max_len, info->rx_max_time);

    data_length_req = false;
    k_sem_give(&performance_test_sem);
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
        shell_error(shell, "Failed to get connection info %d", err);
        return err;
    }

    if (info.role != BT_CONN_ROLE_CENTRAL)
    {
        shell_error(shell, "'run' command shall be executed only on the master board");
    }

    err = bt_conn_le_phy_update(default_conn, phy);
    if (err)
    {
        shell_error(shell, "PHY update failed: %d\n", err);
        return err;
    }

    shell_print(shell, "PHY update pending");
    err = k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);
    if (err)
    {
        shell_error(shell, "PHY update timeout");
        return err;
    }

    if (info.le.data_len->tx_max_len != data_len->tx_max_len)
    {
        data_length_req = true;

        err = bt_conn_le_data_len_update(default_conn, data_len);
        if (err)
        {
            shell_error(shell, "LE data length update failed: %d",
                        err);
            return err;
        }

        shell_print(shell, "LE Data length update pending");
        err = k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);
        if (err)
        {
            shell_error(shell, "LE Data Length update timeout");
            return err;
        }
    }

    if (info.le.interval != conn_param->interval_max)
    {
        err = bt_conn_le_param_update(default_conn, conn_param);
        if (err)
        {
            shell_error(shell, "Connection parameters update failed: %d", err);
            return err;
        }

        shell_print(shell, "Connection parameters update pending");
        err = k_sem_take(&performance_test_sem, PERF_TEST_CONFIG_TIMEOUT);
        if (err)
        {
            shell_error(shell, "Connection parameters update timeout");
            return err;
        }
    }

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

    printk("Starting Bluetooth Performance test example\n");

    bt_conn_cb_register(&conn_callbacks);

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    scan_init();

    err = bt_performance_test_init(&performance_test, &performance_test_cb);
    if (err)
    {
        printk("Performance test service initialization failed.\n");
        return;
    }
}