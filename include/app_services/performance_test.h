/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @defgroup bt_performance_test Bluetooth LE GATT Performance test Service API
 * @{
 * @brief API for the Bluetooth LE GATT Performance test Service.
 */

#ifndef BT_PERF_TEST_H_
#define BT_PERF_TEST_H_

#include <bluetooth/uuid.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Performance test metrics. */
    struct bt_performance_test_metrics
    {
        /** Number of GATT writes received. */
        uint32_t write_count;

        /** Number of bytes received. */
        uint32_t write_len;

        /** Transfer speed in bits per second. */
        uint32_t write_rate;
    };

    /** @brief Performance test callback structure. */
    struct bt_performance_test_cb
    {
        /** @brief Data read callback.
         *
         * This function is called when data has been read from the
         * Performance test Characteristic.
         *
         * @param[in] met Performance test metrics.
         *
         * @retval BT_GATT_ITER_CONTINUE To keep notifications enabled.
         * @retval BT_GATT_ITER_STOP To disable notifications.
         */
        uint8_t (*data_read)(const struct bt_performance_test_metrics *met);

        /** @brief Data received callback.
         *
         * This function is called when data has been received by the
         * Performance test Characteristic.
         *
         * @param[in] met Performance test metrics.
         */
        void (*data_received)(const struct bt_performance_test_metrics *met);

        /** @brief Data send callback.
         *
         * This function is called when data has been sent to the
         * Performance test Characteristic.
         *
         * @param[in] met Performance test metrics.
         */
        void (*data_send)(const struct bt_performance_test_metrics *met);
    };

    /** @brief Performance test structure. */
    struct bt_performance_test
    {
        /** Performance test Characteristic handle. */
        uint16_t char_handle;

        /** GATT read parameters for the Performance test Characteristic. */
        struct bt_gatt_read_params read_params;

        /** Performance test callback structure. */
        struct bt_performance_test_cb *cb;

        /** Connection object. */
        struct bt_conn *conn;
    };

/** @brief Performance test Characteristic UUID. */
#define BT_UUID_PERF_TEST_CHAR BT_UUID_DECLARE_16(0x1524)

#define BT_UUID_PERF_TEST_VAL \
    BT_UUID_128_ENCODE(0x0483dadd, 0x6c9d, 0x6ca9, 0x5d41, 0x03ad4fff4abb)

/** @brief Performance test Service UUID. */
#define BT_UUID_PERF_TEST \
    BT_UUID_DECLARE_128(BT_UUID_PERF_TEST_VAL)

    /** @brief Initialize the GATT Performance test Service.
     *
     *  @param[in] performance_test Performance test Service instance.
     *  @param[in] cb Callbacks.
     *
     *  @retval 0 If the operation was successful.
     *            Otherwise, a negative error code is returned.
     */
    int bt_performance_test_init(struct bt_performance_test *performance_test,
                                 const struct bt_performance_test_cb *cb);

    /** @brief Assign handles to the Performance test Service instance.
     *
     * This function should be called when a link with a peer has been established,
     * to associate the link to this instance of the module. This makes it
     * possible to handle several links and associate each link to a particular
     * instance of this module. The GATT attribute handles are provided by the
     * GATT Discovery Manager.
     *
     * @param[in] dm Discovery object.
     * @param[in,out] performance_test Performance test Service instance.
     *
     * @retval 0 If the operation was successful.
     *           Otherwise, a negative error code is returned.
     * @retval (-ENOTSUP) Special error code used when the UUID
     *         of the service does not match the expected UUID.
     */
    int bt_performance_test_handles_assign(struct bt_gatt_dm *dm,
                                           struct bt_performance_test *performance_test);

    /** @brief Read data from the server.
     *
     *  @note This procedure is asynchronous.
     *
     *  @param[in] performance_test Performance test Service instance.
     *
     *  @retval 0 If the operation was successful.
     *            Otherwise, a negative error code is returned.
     */
    int bt_performance_test_read(struct bt_performance_test *performance_test);

    /** @brief Write data to the server.
     *
     *  @param[in] performance_test Performance test Service instance.
     *  @param[in] data Data.
     *  @param[in] len Data length.
     *
     *  @retval 0 If the operation was successful.
     *            Otherwise, a negative error code is returned.
     */
    int bt_performance_test_write(struct bt_performance_test *performance_test,
                                  const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_PERF_TEST_H_ */