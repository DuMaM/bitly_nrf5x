#ifdef CONFIG_ADS129x_PIPE

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
#include <cmd.h>
#include <cmd_run.h>

#include <app_utils.h>
#include <spi_adc.h>

LOG_MODULE_DECLARE(ecg);

// handle data stream
/*
 * buffer should have at least 145 times size of package
 * to allow for storing one 35ms long connection event
 *
 * We dont support bigger values
 * also 15 packages is added as protection for data overwrites
 */
K_PIPE_DEFINE(ads129x_pipe, ADS129x_DATA_BUFFER_SIZE * 3 * 160, 4);

/*
 * in this spi buffer not whole data is stored
 * it only uses to map spi data map
 */
pipe_packet_u tx_data;
static struct spi_buf ads129x_rx_bufs[] = {{.buf = tx_data.packet.leads._buffer, .len = ADS129X_SPI_PACKAGE_SIZE}};
static struct spi_buf_set ads129x_rx = {.buffers = ads129x_rx_bufs, .count = 1};

int8_t ads129x_reset_data(void)
{
    utils_reset_timestamp();
    k_pipe_flush(&ads129x_pipe);
    return 0;
}

int32_t ads129x_get_data(uint8_t *load_data, int32_t size)
{
    int rc = 0;
    int32_t total = size;
    size_t bytes_read;
    size_t min_size = sizeof(pipe_packet_u);

    while (1)
    {
        /**
         * sometimes we run this function to fetch last remaining
         * chunk of data, this can be smaller that pipe package data
         * so this will allow us to pull remaining one
         *
         * there is a second condition when we reduced
         * our size because pipe was not able to give all data at once
         */
        if (size < min_size)
        {
            min_size = size;
        }

        rc = k_pipe_get(&ads129x_pipe, load_data, size, &bytes_read, min_size, K_USEC(250));

        if (rc == -EINVAL)
        {
            LOG_ERR("Bad input data: size=%d, min_size=%d, read=%d", size, min_size, bytes_read);
            return -1;
        }
        else if ((rc < 0) || (bytes_read < min_size))
        {
            LOG_DBG("Waiting period timed out; between zero and min_xfer minus one data bytes were read. %d", rc);
            // size -= bytes_read;
            // load_data += bytes_read;
        }
        else if (bytes_read < size)
        {
            LOG_DBG("Buffer is not fully filled - moving");
            size -= bytes_read;
            load_data += bytes_read;
        }
        else
        {
            /* All data was received */
            break;
        }
    }

    return total;
}

void ads129x_set_data()
{
    /* track status of buffers */
    static size_t bytes_written = 0;

    /* add timestamp */
    utils_write_timestamp(tx_data.buffer);

    /* do processing */
    /* NOTE: Work directly on a ring buffer memory */
    int ret = spi_read(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, &ads129x_rx);
    if (!ret)
    {
        /* add missing leads */
        ads129x_load_augmented_leads(tx_data.packet.leads._buffer);

        ads129x_dump_data(tx_data.packet.leads._buffer);
        /* send data to the consumers */
        k_pipe_put(&ads129x_pipe, &tx_data.buffer, ADS129x_DATA_BUFFER_SIZE, &bytes_written, sizeof(pipe_packet_u), K_NO_WAIT);
    }
}


void ads129x_th(void)
{
    /* setup ecg */
    ads129x_setup();

    for (;;)
    {
        /*
         * Wait for semaphore from ISR; if acquired, do related work, then
         * go to next loop iteration (the semaphore might have been given
         * again); else, make the CPU idle.
         */
        if (k_sem_take(&ads129x_new_data, K_FOREVER) == 0)
        {
            ads129x_set_data();
        }
    }
}

K_THREAD_DEFINE(thread_ads129x, ADS129X_STACKSIZE, ads129x_th, NULL, NULL, NULL, ADS129X_PRIORITY, K_ESSENTIAL, 0);

#endif