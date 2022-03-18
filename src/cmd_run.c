#include <cmds.h>
#include <bt_test.h>
#include "img_file.h"
#include <sys/byteorder.h>

extern test_params_t test_params;
extern struct bt_conn *default_conn;
/* a dummy data buffer */
static uint8_t dummy[256];

static void get_rssi_power(struct bt_conn *conn)
{
	uint16_t conn_handle;

	int err = bt_hci_get_conn_handle(conn, &conn_handle);

	if (err) {
		printk("No connection handle (err %d)", err);
		return;
	}

	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;
	struct net_buf *buf;
	struct net_buf *rsp = NULL;

	buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
	if (!buf) {
		printk("Cannot allocate buffer to get RSSI power");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(conn_handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		printk("Get rssi power value (err: %d)\n", err);
	} else {
		rp = (struct bt_hci_rp_read_rssi *)(rsp->data);
		printk("RSSI power returned by command: %"PRId8"\n", rp->rssi);
	}

	if (rsp) {
		net_buf_unref(rsp);
	}
	
}

static void get_tx_power(struct bt_conn *conn) {
    struct bt_conn_le_tx_power tx_power = {.phy = 0};
    int error = bt_conn_le_get_tx_power_level(conn, &tx_power);
    if ( !error ) {
        printk("TX power returned by command: %"PRId8"\n", tx_power.current_level);
    }
}

static uint8_t throughput_read(const struct bt_throughput_metrics *met)
{
    printk("[peer] received %u bytes (%u KB)"
           " in %u GATT writes at %u bps\n",
           met->write_len, met->write_len / 1024, met->write_count,
           met->write_rate);

    
    k_sem_give(&throughput_sem);

    return BT_GATT_ITER_STOP;
}

static void throughput_received(const struct bt_throughput_metrics *met)
{
    static uint32_t kb = 0;
    static bool enter = true;

    /* init case for new package sent */
    if (met->write_len == 0) {
        kb = 0;
        printk("\n");
        enter = false;
        return;
    }

    if ((met->write_len / 1024) != kb) {
        kb = (met->write_len / 1024);
        printk("=");
        enter = true;
    }

    /* add formatting */
    if (! (kb % 64) && enter) {
        printk("\n");
        get_rssi_power(default_conn);
	    get_tx_power(default_conn);
        enter = false;
    }
}

static void throughput_send(const struct bt_throughput_metrics *met)
{
    printk("\n[local] received %u bytes (%u KB)"
        " in %u GATT writes at %u bps\n",
        met->write_len, met->write_len / 1024,
        met->write_count, met->write_rate);
}

const struct bt_throughput_cb throughput_cb = {
    .data_read = throughput_read,
    .data_received = throughput_received,
    .data_send = throughput_send
};

int test_init(                  const struct shell *shell,
                                const struct bt_le_conn_param *conn_param,
                                const struct bt_conn_le_phy_param *phy,
                                const struct bt_conn_le_data_len_param *data_len) {
    int err;

    if (!getSettings()) {
        shell_error(shell, "Device is disconnected %s",
                "Connect to the peer device before running test");
        return -EFAULT;
    }

    if (!isTestReady()) {
        shell_error(shell, "Device is not ready."
            "Please wait for the service discovery and MTU exchange end");
        return 0;
    }

    shell_print(shell, "\n==== Starting throughput test ====");

    err = connection_configuration_set(shell, conn_param, phy, data_len);
    if (err) {
        return err;
    }

    /* reset peer metrics */
    err = bt_throughput_write(&throughput, dummy, 1);
    if (err) {
        shell_error(shell, "Reset peer metrics failed.");
        return err;
    }

    /* Make sure that all BLE procedures are finished. */
    k_sleep(K_MSEC(500));
    return 0;
}


int test_run(const struct shell *shell,
         const struct bt_le_conn_param *conn_param,
         const struct bt_conn_le_phy_param *phy,
         const struct bt_conn_le_data_len_param *data_len)
{
    int64_t stamp;
    int64_t delta;
    uint32_t prog = 0;
    uint32_t data = 0;
    int err;

    err = test_init(shell, conn_param, phy, data_len);
    if (err) {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }

    /* get cycle stamp */
    stamp = k_uptime_get_32();

    while (prog < IMG_SIZE) {
        err = bt_throughput_write(&throughput, dummy, 244);
        if (err) {
            shell_error(shell, "GATT write failed (err %d)", err);
            break;
        }

        /* print graphics */
        printk("%c", img[prog / IMG_X][prog % IMG_X]);
        data += 244;
        prog++;
    }

    delta = k_uptime_delta(&stamp);

    printk("\nDone\n");
    printk("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps\n",
           data, data / 1024, delta, ((uint64_t)data * 8 / delta));

    /* read back char from peer */
    err = bt_throughput_read(&throughput);
    if (err) {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }

    k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);

    instruction_print();

    return 0;
}


static int test_run_cmd(const struct shell *shell, size_t argc,
            char **argv)
{
    return test_run(shell, test_params.conn_param, test_params.phy,
            test_params.data_len);
}

int test_run_ber_alternating(   const struct shell *shell,
                                const struct bt_le_conn_param *conn_param,
                                const struct bt_conn_le_phy_param *phy,
                                const struct bt_conn_le_data_len_param *data_len)
{
    int64_t stamp;
    int64_t delta;
    uint32_t prog = 0;
    uint32_t data = 0;
    int err;

    err = test_init(shell, conn_param, phy, data_len);
    if (err) {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }
    
    for (int i = 0; i < 256; i++) {
        if (i % 4 > 1) {
            dummy[i] = '1';
        } else {
            dummy[i] = '0';
        }
    }

    /* get cycle stamp */
    stamp = k_uptime_get_32();
    
    while (prog < IMG_SIZE) {
        err = bt_throughput_write(&throughput, dummy, 256);
        if (err) {
            shell_error(shell, "GATT write failed (err %d)", err);
            break;
        }

        /* print graphics */
        printk("%c", img[prog / IMG_X][prog % IMG_X]);
        data += 244;
        prog++;
    }

    delta = k_uptime_delta(&stamp);

    printk("\nDone\n");
    printk("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps\n",
           data, data / 1024, delta, ((uint64_t)data * 8 / delta));

    /* read back char from peer */
    err = bt_throughput_read(&throughput);
    if (err) {
        shell_error(shell, "GATT read failed (err %d)", err);
        return err;
    }

    k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);

    instruction_print();

    return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_bt_test,
    SHELL_CMD(ber_alternating,  NULL, "Tests ber signal with pattern |11|00|11|00", default_cmd),
    SHELL_CMD(ber_oppsed,       NULL, "Tests ber signal with pattern |10|10|10|10", default_cmd),
    SHELL_CMD(analog_sim,       NULL, "Tests with simulated ECC signal", default_cmd),
    SHELL_CMD(analog,           NULL, "Tests with ECC signal", default_cmd),
    SHELL_CMD(test_picture,     NULL, "Tests transition with example picture", test_run_cmd),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(run, &sub_bt_test, "Run the test", default_cmd);
