#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/types.h>

#include <stdlib.h>
#include <stddef.h>

// todo fix this
#include <performance_test.h>
#include <bt_test.h>

#include <spi_adc.h>


LOG_MODULE_DECLARE(ads129x_log);


#ifdef CONFIG_ADS129x_RINGBUFF
// ###########
// # BUFFER
// ###########

// stores n numbers of spi data packet
// ring buffer size should be bigger then 251 data len
// here is set to be 3 times bigger
#define ADS129X_RING_BUFFER_PACKET ((uint8_t)(251 / ADS129x_DATA_BUFFER_SIZE))
#define ADS129X_RING_BUFFER_SIZE (ADS129X_RING_BUFFER_PACKET * ADS129x_DATA_BUFFER_SIZE * 3)

// handle simple data requests
K_SEM_DEFINE(ads129x_ring_buffer_sem, 1, 1);
RING_BUF_DECLARE(ads129x_ring_buffer, ADS129X_RING_BUFFER_SIZE * 12);

int wait_for_finish()
{
    return k_sem_take(&ads129x_ring_buffer_sem, K_FOREVER);
}

/*
 * in this spi buffer not whole data is stored
 * it only uses to map spi data map
 */
pipe_packet_u tx_data;
static struct spi_buf ads129x_rx_bufs[] = {{.buf = tx_data.packet.leads._buffer, .len = ADS129X_SPI_PACKAGE_SIZE}};
static struct spi_buf_set ads129x_rx = {.buffers = ads129x_rx_bufs, .count = 1};


/**
 * Read data by command; supports multiple read back.
 * This command assumes that continuous read is disabled
 */
void ads129x_read_data(void)
{
    LOG_DBG("CMD: Read Data");
    if (ring_buf_capacity_get(&ads129x_ring_buffer) >= ADS129x_DATA_BUFFER_SIZE)
    {
        LOG_ERR("CMD: READ_DATA - There is no space in buffer");
        return;
    }

    uint8_t *data = NULL;

    /* Allocate buffer within a ring buffer memory. */
    uint16_t size = ring_buf_put_claim(&ads129x_ring_buffer, &data, ADS129x_DATA_BUFFER_SIZE);

    /* add timestamp */
    ads129x_write_timestamp(data);

    /* do processing */
    /* NOTE: Work directly on a ring buffer memory */
    int ret = ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, ADS129X_CMD_RDATA, data, size);
    if (!ret)
    {

        /* Indicate amount of valid data. rx_size can be equal or less than size. */
        ads129x_load_augmented_leads(data);
    }

    /* Indicate amount of valid data. rx_size can be equal or less than size. */
    ring_buf_put_finish(&ads129x_ring_buffer, size);
}


int8_t ads129x_reset_data(void)
{
    ads129x_config.timestamp = k_uptime_get();

    ring_buf_reset(&ads129x_ring_buffer);
    return 0;
}

/**
 * Read data by command; supports multiple read back.
 */
void ads129x_read_data_continuous(void)
{
    if (ring_buf_space_get(&ads129x_ring_buffer) < ADS129x_DATA_BUFFER_SIZE)
    {
        return;
    }

    uint8_t *data = NULL;

    /* Allocate buffer within a ring buffer memory. */
    uint16_t size = ring_buf_put_claim(&ads129x_ring_buffer, &data, ADS129x_DATA_BUFFER_SIZE);

    /* add timestamp */
    ads129x_write_timestamp(data);

    /* do processing */
    /* NOTE: Work directly on a ring buffer memory */
    int ret = spi_read(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, &ads129x_rx);
    if (!ret)
    {
        /* add missing leads */
        ads129x_load_augmented_leads(data);
    }
    /* Indicate amount of valid data. rx_size can be equal or less than size. */
    ring_buf_put_finish(&ads129x_ring_buffer, size);
}

/**
 * Read data by command; supports multiple read back.
 */
uint32_t ads129x_write_data_continuous(uint8_t **buffer, uint32_t size)
{
    if (ring_buf_size_get(&ads129x_ring_buffer) < size)
    {
        return 0;
    }

    /* Allocate buffer within a ring buffer memory. */
    return ring_buf_get_claim(&ads129x_ring_buffer, buffer, size);
}

void ads129x_write_data_continuous_fin(uint32_t size)
{
    /* do processing */
    /* NOTE: Work directly on a ring buffer memory */
    /* Indicate amount of valid data. rx_size can be equal or less than size. */
    ring_buf_get_finish(&ads129x_ring_buffer, size);
}

uint32_t get_ready_data(uint16_t mtu_size, uint32_t _bytes_to_send)
{
    uint32_t analog_data_size = _bytes_to_send;
    if (mtu_size <= analog_data_size)
    {
        analog_data_size = mtu_size;
    }

    uint32_t buffer_size = ring_buf_size_get(&ads129x_ring_buffer);
    if (buffer_size >= analog_data_size)
    {
        return analog_data_size;
    }
    else
    {
        return 0;
    }
}

static uint32_t send_test_ecg_alt(uint32_t _bytes_to_send)
{
    uint8_t *analog_data_ptr = NULL;
    uint32_t analog_data_size = 0;
    uint32_t get_size = 0;
    int err = 0;

    // LOG_INF("Sending %"PRIu32" bytes (value after rounding to max packet size)", _bytes_to_send);

    if (_bytes_to_send)
    {
        analog_data_size = _bytes_to_send;
        if (test_params.data_len->tx_max_len - 7 <= analog_data_size)
        {
            analog_data_size = test_params.data_len->tx_max_len - 7;
        }

        get_size = ads129x_write_data_continuous(&analog_data_ptr, analog_data_size);
        if (get_size < analog_data_size)
        {
            return 0;
        }

        err = bt_performance_test_write(&performance_test, analog_data_ptr, get_size);
        if (err)
        {
            LOG_ERR("GATT write failed (err %d)", err);
            return 0;
        }

        ads129x_write_data_continuous_fin(get_size);
    }
    return get_size;
}


void ads129x_th(void)
{
    uint32_t data_size_drdy = 0;

    /* setup ecg */
    ads129x_setup();

    for (;;)
    {
        data_size_drdy = get_ready_data(251 - 4, ads129x_config.bytes_to_send);

        //data_size_drdy = get_ready_data(test_params.data_len->tx_max_len - 4, ads129x.bytes_to_send);
        if ((!k_sem_count_get(&ads129x_new_data)) && (data_size_drdy))
        {
            /*
             * new data arrival is best place
             * to send them over ble
             * mostly because there is still a lot of interval between next call
             */
            if (ads129x_config.bytes_to_send > 0)
            {
                ads129x_config.bytes_to_send -= send_test_ecg_alt(ads129x_config.bytes_to_send);

                if (ads129x_config.bytes_to_send == 0)
                {
                    k_sem_give(&ads129x_ring_buffer_sem);
                }
            }
        }

        /*
         * Wait for semaphore from ISR; if acquired, do related work, then
         * go to next loop iteration (the semaphore might have been given
         * again); else, make the CPU idle.
         */
        else if (k_sem_take(&ads129x_new_data, K_FOREVER) == 0)
        {

            /*
             * get new data to spi
             */
            ads129x_read_data_continuous();
        }
    }
}

K_THREAD_DEFINE(thread_ads129x, ADS129X_STACKSIZE, ads129x_th, NULL, NULL, NULL, ADS129X_PRIORITY, K_ESSENTIAL, 0);

#endif