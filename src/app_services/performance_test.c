/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <kernel.h>
#include <sys/printk.h>
#include <string.h>
#include <zephyr/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include <performance_test.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(bt_performance_test, CONFIG_BT_PERF_TEST_LOG_LEVEL);

static struct bt_performance_test_metrics met;
static const struct bt_performance_test_cb *callbacks;

static uint8_t read_fn(struct bt_conn *conn, uint8_t err,
                       struct bt_gatt_read_params *params, const void *data,
                       uint16_t len)
{
    struct bt_performance_test_metrics metrics;

    memset(&metrics, 0, sizeof(struct bt_performance_test_metrics));

    if (data)
    {
        len = MIN(len, sizeof(struct bt_performance_test_metrics));
        memcpy(&metrics, data, len);

        if (callbacks && callbacks->data_read)
        {
            return callbacks->data_read(&metrics);
        }
    }

    return BT_GATT_ITER_STOP;
}

static ssize_t write_callback(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr, const void *buf,
                              uint16_t len, uint16_t offset, uint8_t flags)
{
    static uint32_t clock_cycles;
    static uint32_t kb;

    uint64_t delta;

    struct bt_performance_test_metrics *met_data = attr->user_data;

    delta = k_cycle_get_32() - clock_cycles;
    delta = k_cyc_to_ns_floor64(delta);

    if (len == 1)
    {
        /* reset metrics */
        kb = 0;
        met_data->write_count = 0;
        met_data->write_len = 0;
        met_data->write_rate = 0;
        clock_cycles = k_cycle_get_32();
    }
    else
    {
        met_data->write_count++;
        met_data->write_len += len;
        met_data->write_rate =
            ((uint64_t)met_data->write_len << 3) * 1000000000 / delta;
    }

    LOG_DBG("Received data.");

    if (callbacks && callbacks->data_received)
    {
        callbacks->data_received(met_data);
    }

    return len;
}

static ssize_t read_callback(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr, void *buf,
                             uint16_t len, uint16_t offset)
{
    const struct bt_performance_test_metrics *metrics = attr->user_data;

    len = MIN(sizeof(struct bt_performance_test_metrics), len);

    if (callbacks && callbacks->data_send)
    {
        callbacks->data_send(metrics);
    }

    LOG_DBG("Data send.");

    return bt_gatt_attr_read(
        conn, attr, buf, len, offset, attr->user_data, len);
}

BT_GATT_SERVICE_DEFINE(performance_test_svc,
                       BT_GATT_PRIMARY_SERVICE(BT_UUID_PERF_TEST),
                       BT_GATT_CHARACTERISTIC(BT_UUID_PERF_TEST_CHAR,
                                              BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                                              read_callback, write_callback, &met), );

int bt_performance_test_init(struct bt_performance_test *performance_test,
                             const struct bt_performance_test_cb *cb)
{
    if (!performance_test || !cb)
    {
        return -EINVAL;
    }

    callbacks = cb;

    return 0;
}

int bt_performance_test_handles_assign(struct bt_gatt_dm *dm,
                                       struct bt_performance_test *performance_test)
{
    const struct bt_gatt_dm_attr *gatt_service_attr =
        bt_gatt_dm_service_get(dm);
    const struct bt_gatt_service_val *gatt_service =
        bt_gatt_dm_attr_service_val(gatt_service_attr);
    const struct bt_gatt_dm_attr *gatt_chrc;
    const struct bt_gatt_dm_attr *gatt_desc;

    if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_PERF_TEST))
    {
        return -ENOTSUP;
    }

    LOG_DBG("Getting handles from performance test service.");

    /* Performance test Characteristic. */
    gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_PERF_TEST_CHAR);
    if (!gatt_chrc)
    {
        LOG_ERR("Missing performance test characteristic.");
    }

    gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc,
                                        BT_UUID_PERF_TEST_CHAR);
    if (!gatt_desc)
    {
        LOG_ERR("Missing performance test characteristic value descriptor");
        return -EINVAL;
    }

    LOG_DBG("Found handle for Performance test characteristic.");
    performance_test->char_handle = gatt_desc->handle;

    /* Assign connection object. */
    performance_test->conn = bt_gatt_dm_conn_get(dm);
    return 0;
}

int bt_performance_test_read(struct bt_performance_test *performance_test)
{
    int err;

    performance_test->read_params.single.handle = performance_test->char_handle;
    performance_test->read_params.single.offset = 0;
    performance_test->read_params.handle_count = 1;
    performance_test->read_params.func = read_fn;

    err = bt_gatt_read(performance_test->conn, &performance_test->read_params);
    if (err)
    {
        LOG_ERR("Characteristic read failed.");
    }

    return err;
}

int bt_performance_test_write(struct bt_performance_test *performance_test,
                              const uint8_t *data, uint16_t len)
{
    return bt_gatt_write_without_response(performance_test->conn,
                                          performance_test->char_handle,
                                          data, len, false);
}
