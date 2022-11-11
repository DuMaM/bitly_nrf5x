/**
 * ADS129X.cpp
 *
 * Arduino library for the TI ADS129X series of analog-front-ends for
 * biopotential measurements (EMG/EKG/EEG).
 *
 * This library offers two modes of operation: polling and interrupt.
 * Polling mode should only be used in situations where multiple devices
 * share the SPI bus. Interrupt mode is much faster (8kSPS on a Teensy 3.1),
 * but starts receiving immediately when DRDY goes high.
 * The API is the same for both modes. To activate polling mode add
 *     #define ADS129X_POLLING
 * as first line to your sketch.
 *
 * Based on code by Conor Russomanno (https://github.com/conorrussomanno/ADS1299)
 * Modified by Ferdinand Keil
 */

#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

/**
 * WARN: ADS129x is using BIG_ENDIAN
 * Where most of arms is using a little_endian
 */
#include <spi_adc.h>
#include <app_utils.h>

#ifdef CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP

/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 8
K_SEM_DEFINE(ads129x_new_data, 0, 1);
static bool ads129x_print_data = false;

LOG_MODULE_REGISTER(ads129x_log, LOG_LEVEL_INF);
#define ADS_CLK_PERIOD_US 30
static void ads129x_drdy_init_callback(void);
static void ads129x_drdy_callback_deinit(void);

// start pin
#define START_NODE DT_NODELABEL(ads129x_start)
#if defined(START_NODE)
#define START_FLAGS DT_GPIO_FLAGS(START_NODE, gpios)
#define START_PIN DT_GPIO_PIN(START_NODE, gpios)
struct gpio_dt_spec start_spec = GPIO_DT_SPEC_GET_OR(START_NODE, gpios, {0});
#endif

// reset pin
#define RESET_NODE DT_NODELABEL(ads129x_reset)
#define RESET_FLAGS DT_GPIO_FLAGS(RESET_NODE, gpios)
#define RESET_PIN DT_GPIO_PIN(RESET_NODE, gpios)
struct gpio_dt_spec reset_spec = GPIO_DT_SPEC_GET_OR(RESET_NODE, gpios, {0});

// drdy pin
#define DRDY_NODE DT_NODELABEL(ads129x_drdy)
#define DRDY_FLAGS DT_GPIO_FLAGS(DRDY_NODE, gpios)
#define DRDY_PIN DT_GPIO_PIN(DRDY_NODE, gpios)
struct gpio_dt_spec drdy_spec = GPIO_DT_SPEC_GET_OR(DRDY_NODE, gpios, {0});

// spi but macros and objects
#define SPI_NODE DT_NODELABEL(nrf53_spi)

// stores n numbers of spi data packet
// ring buffer size should be bigger then 251 data len
// here is set to be 3 times bigger
#define ADS129X_RING_BUFFER_PACKET ((uint8_t)(251 / ADS129x_DATA_BUFFER_SIZE))
#define ADS129X_RING_BUFFER_SIZE (ADS129X_RING_BUFFER_PACKET * ADS129x_DATA_BUFFER_SIZE * 3)
K_SEM_DEFINE(ads129x_ring_buffer_rdy, 0, ADS129X_RING_BUFFER_PACKET);
K_MUTEX_DEFINE(ads129x_ring_buffer_mutex);
RING_BUF_DECLARE(ads129x_ring_buffer, ADS129X_RING_BUFFER_SIZE);

const struct device *ads129x_spi = DEVICE_DT_GET(SPI_NODE);
const struct spi_cs_control ads129x_cs_ctrl = {
    .gpio.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .delay = ADS129X_SPI_CLOCK_DELAY,
    .gpio.pin = 4,
    .gpio.dt_flags = GPIO_ACTIVE_LOW};

// arduino lib was working with SPI_MODE1
// what means
// Clock Polarity (CPOL)    Clock Phase (CPHA)	Output Edge     Data Capture
// 0                        1                   Rising          Falling
struct spi_config ads129x_spi_cfg = {
    .frequency = ADS129X_SPI_CLOCK_SPEED,
    .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .cs = &ads129x_cs_ctrl};

// checks
#if !DT_NODE_HAS_STATUS(DRDY_NODE, okay)
#error "Unsupported board: drdy devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(RESET_NODE, okay)
#error "Unsupported board: reset devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SPI_NODE, okay)
#error "Unsupported board: spi2 devicetree alias is not defined"
#endif

/**
 * @brief Set or clear a bit depending on a boolean value
 *
 * The argument @p var is a variable whose value is written to as a
 * side effect.
 *
 * @param var Variable to be altered
 * @param bit Bit number
 * @param set if 0, clears @p bit in @p var; any other value sets @p bit
 */
#define WRITE_BIT_VAL(var, bit, set) \
    ((var) = (set) ? ((var) | bit) : ((var) & ~bit))

static inline int32_t ads129x_get_leadI(uint8_t *data_buffer)
{
    return conv_u24_to_i32(conv_raw_to_u24(data_buffer, ADS129x_LEAD1_OFFSET));
}

static inline int32_t ads129x_get_leadII(uint8_t *data_buffer)
{
    return conv_u24_to_i32(conv_raw_to_u24(data_buffer, ADS129x_LEAD2_OFFSET));
}

static inline uint32_t ads129x_get_leadIII(int32_t lead1, int32_t lead2)
{
    return conv_i32_to_u24(lead1 - lead2);
}

static inline uint32_t ads129x_get_aVR(int32_t lead1, int32_t lead2)
{
    return conv_i32_to_u24((lead1 + lead2) / -2);
}

static inline uint32_t ads129x_get_aVL(int32_t lead1, int32_t lead2)
{
    return conv_i32_to_u24((lead2 - lead1) / 2);
}

static inline uint32_t ads129x_get_aVF(int32_t lead1, int32_t lead2)
{
    return conv_i32_to_u24((lead2 - lead1) / 2);
}

static int ads129x_access(const struct device *_spi,
                          struct spi_config *_spi_cfg,
                          uint8_t _cmd,
                          uint8_t *_data,
                          uint8_t _len)
{
    uint8_t cmd = _cmd;
    uint8_t n = _len - 1;
    struct spi_buf tx_bufs[] = {
        {.buf = &cmd,
         .len = sizeof(cmd)},
        {.buf = &n,
         .len = sizeof(n)},
        {.buf = _data,
         .len = _len}};
    struct spi_buf_set tx = {.buffers = tx_bufs};

    if (cmd & ADS129X_CMD_RDATA)
    {
        tx.count = 1;
        struct spi_buf rx_bufs[] = {
            // skip response for cmd
            {.buf = NULL, .len = sizeof(cmd)},
            // get response after sending value number of regs
            {.buf = _data,
             .len = _len}};
        struct spi_buf_set rx = {.buffers = rx_bufs, .count = sizeof(rx_bufs) / sizeof(rx_bufs[0])};
        return spi_transceive(_spi, _spi_cfg, &tx, &rx);
    }
    else if (cmd & ADS129X_CMD_RREG)
    {
        tx.count = 2;
        struct spi_buf rx_bufs[] = {
            // skip response for cmd
            {.buf = NULL, .len = sizeof(cmd)},
            // get response after sending value number of regs
            {.buf = NULL, .len = sizeof(n)},
            {.buf = _data,
             .len = _len}};
        struct spi_buf_set rx = {.buffers = rx_bufs, .count = 3};
        return spi_transceive(ads129x_spi, &ads129x_spi_cfg, &tx, &rx);
    }
    else if (cmd & ADS129X_CMD_WREG)
    {
        tx.count = 3;
        return spi_write(ads129x_spi, &ads129x_spi_cfg, &tx);
    }
    else
    {
        tx.count = 1;
        return spi_write(ads129x_spi, &ads129x_spi_cfg, &tx);
    }
}

// System Commands

/**
 * Exit Standby Mode.
 */
void ads129x_wakeup(void)
{
    LOG_DBG("CMD: Wakeup");
    ads129x_access(ads129x_spi, &ads129x_spi_cfg, ADS129X_CMD_WAKEUP, NULL, 0);
}

/**
 * Enter Standby Mode.
 */
void ads129x_standby(void)
{
    LOG_DBG("CMD: Standby");
    ads129x_access(ads129x_spi, &ads129x_spi_cfg, ADS129X_CMD_STANDBY, NULL, 0);
}

/**
 * Reset Registers to Default Values.
 */
void ads129x_reset(void)
{
    LOG_DBG("CMD: Reset");
    ads129x_access(ads129x_spi, &ads129x_spi_cfg, ADS129X_CMD_RESET, NULL, 0);
    // must wait 18 tCLK cycles to execute this command (Datasheet, pg. 38)
    k_usleep(ADS129X_SPI_CLOCK_DELAY * 5);
}

/**
 * Start/restart (synchronize) conversions.
 */
void ads129x_start(void)
{
    LOG_DBG("CMD: Start");
    ads129x_access(ads129x_spi, &ads129x_spi_cfg, ADS129X_CMD_START, NULL, 0);
    ads129x_drdy_init_callback();
}

/**
 * Stop conversion.
 */
void ads129x_stop()
{
    LOG_DBG("CMD: Stop");
    ads129x_drdy_callback_deinit();
    ads129x_access(ads129x_spi, &ads129x_spi_cfg, ADS129X_CMD_STOP, NULL, 0);
}

/**
 * Enable Read Data Continuous mode (default).
 */
void ads129x_rdatac(void)
{
    LOG_DBG("CMD: Read Data Continuous");
    ads129x_access(ads129x_spi, &ads129x_spi_cfg, ADS129X_CMD_RDATAC, NULL, 0);
}

/**
 * Stop Read Data Continuously mode.
 */
void ads129x_sdatac(void)
{
    LOG_DBG("CMD: Stop Read Data Continuous");
    ads129x_access(ads129x_spi, &ads129x_spi_cfg, ADS129X_CMD_SDATAC, NULL, 0);
}

/**
 * Read data by command; supports multiple read back.
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
    k_mutex_lock(&ads129x_ring_buffer_mutex, K_MSEC(100));

    /* Allocate buffer within a ring buffer memory. */
    uint16_t size = ring_buf_put_claim(&ads129x_ring_buffer, &data, ADS129x_DATA_BUFFER_SIZE);

    /* do processing */
    /* NOTE: Work directly on a ring buffer memory */
    int ret = ads129x_access(ads129x_spi, &ads129x_spi_cfg, ADS129X_CMD_RDATA, data, size);
    if (!ret)
    {
        /* add missing leads */
        int32_t lead1 = ads129x_get_leadI(data);
        int32_t lead2 = ads129x_get_leadII(data);
        conv_u24_to_raw(ads129x_get_leadIII(lead1, lead2), data, ADS129x_LEAD3_OFFSET);
        conv_u24_to_raw(ads129x_get_aVR(lead1, lead2), data, ADS129x_AVR_OFFSET);
        conv_u24_to_raw(ads129x_get_aVL(lead1, lead2), data, ADS129x_AVL_OFFSET);
        conv_u24_to_raw(ads129x_get_aVF(lead1, lead2), data, ADS129x_AVF_OFFSET);
    }
    /* Indicate amount of valid data. rx_size can be equal or less than size. */
    ring_buf_put_finish(&ads129x_ring_buffer, size);

    k_mutex_unlock(&ads129x_ring_buffer_mutex);
    k_sem_give(&ads129x_ring_buffer_rdy);
}

/**
 * Locks spi and keeps cs low for purpose of sending multiple commands
 */
static void ads129x_lock_spi(void)
{
    WRITE_BIT_VAL(ads129x_spi_cfg.operation, (SPI_LOCK_ON | SPI_HOLD_ON_CS), 1);
}

/**
 * Unlocks spi and release cs
 */
static void ads129x_unlock_spi(void)
{
    spi_release(ads129x_spi, &ads129x_spi_cfg);
    WRITE_BIT_VAL(ads129x_spi_cfg.operation, (SPI_LOCK_ON | SPI_HOLD_ON_CS), 0);
}

/**
 * Read _numRegisters register starting at address _address.
 * @param _address      start address
 * @param _numRegisters number of registers
 * @param data          pointer to data array
 */
int ads129x_read_registers(uint8_t _address, uint8_t _n, uint8_t *_value)
{
    // lock spi and cs
    ads129x_lock_spi();

    // stop continuous read which leads to interferences
    ads129x_sdatac();

    // 001rrrrr; _RREG = 00100000 and _address = rrrrr
    uint8_t opcode1 = ADS129X_CMD_RREG | (_address & 0x1F);
    int ret = ads129x_access(ads129x_spi, &ads129x_spi_cfg, opcode1, _value, _n);
    LOG_DBG("RR: %02X (%" PRIu8 ")", _address & 0x1F, _n);

    // unlock after successful operation
    ads129x_unlock_spi();
    return ret;
}

/**
 * Write register at address _address.
 * @param _address register address
 * @param _value   register value
 */
int ads129x_write_registers(uint8_t _address, uint8_t _n, uint8_t *_value)
{
    // 010wwwww; _WREG = 00100000 and _address = wwwww
    uint8_t opcode1 = ADS129X_CMD_WREG | (_address & 0x1F);
    if (_n == 1)
    {
        LOG_DBG("WR: %02X (data=%" PRIu8 ")", _address & 0x1F, *_value);
    }
    else
    {
        LOG_DBG("WR: %02X (n=%" PRIu8 ")", _address & 0x1F, _n);
    }
    return ads129x_access(ads129x_spi, &ads129x_spi_cfg, opcode1, _value, _n);
}

/**
 * Write register at address _address.
 * @param _address register address
 * @param _value   register value
 */
int ads129x_safe_write_register(uint8_t _address, uint8_t _value)
{
    uint8_t tmp = 0xff;
    ads129x_write_registers(_address, 1, &_value);
    ads129x_read_registers(_address, 1, &tmp);

    if (tmp != _value)
    {
        LOG_ERR("WR: %02X (data=%" PRIu8 ")", _address & 0x1F, _value);
        LOG_ERR("RR: %02X (data=%" PRIu8 ")", _address & 0x1F, tmp);
        return -EIO;
    }
    else
    {
        return 0;
    }
}

/**
 * Read device ID.
 * @return device ID
 */
int ads129x_get_device_id(uint8_t *dev_id)
{
    return ads129x_read_registers(ADS129X_REG_ID, 1, dev_id);
}

/* add gpio callbacks */
static struct gpio_callback drdy_cb_data;
static void drdy_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&ads129x_new_data);
}

static void ads129x_drdy_init_callback(void)
{
    // configure the interrupt on drdy press (pin goes from low to high)
    gpio_pin_interrupt_configure_dt(&drdy_spec, GPIO_INT_EDGE_TO_INACTIVE);

    // setup the drdy change callback
    gpio_init_callback(&drdy_cb_data, drdy_callback, BIT(drdy_spec.pin));
    gpio_add_callback(drdy_spec.port, &drdy_cb_data);
}

void ads129x_drdy_callback_deinit()
{
    gpio_pin_interrupt_configure_dt(&drdy_spec, GPIO_INT_DISABLE);
    gpio_remove_callback(drdy_spec.port, &drdy_cb_data);
}

/**
 * Configure channel _channel.
 * @param _channel   channel (1-8)
 * @param _powerDown power down (true, false)
 * @param _gain      gain setting
 * @param _mux       mux setting
 */
void ads129x_configChannel(uint8_t _channel, bool _powerDown, uint8_t _gain, uint8_t _mux)
{
    uint8_t value = ((_powerDown & 1) << 7) | ((_gain & 7) << 4) | (_mux & 7);
    ads129x_write_registers(ADS129X_REG_CH1SET + (_channel - 1), 1, &value);
}

void ads129x_print(bool _print)
{
    ads129x_print_data = _print;
}

void ads129x_dump_data(uint8_t *input_data)
{
    int32_t data[ADS129X_DATA_NUM];
    char print_buf[ADS129X_DATA_NUM * 10 + 1];

    if (ads129x_print_data)
    {
        uint8_t print_buf_pos = 0;
        memset(data, 0, sizeof(data));

        for (int i = 0; i < ADS129X_DATA_NUM; i++)
        {
            data[i] = conv_u24_to_i32(conv_raw_to_u24(input_data, i * 3));
            print_buf_pos += snprintk(print_buf + print_buf_pos, sizeof(print_buf) - print_buf_pos, "%" PRIi32 " ", data[i]);
        }
        LOG_INF("Data: %s", print_buf);
    }
}

/**
 * Initializes ADS129x library.
 */
void ads129x_init(void)
{
    // SPI Setup
    int ret;
    LOG_INF("ADS129X spi init");
    if (!device_is_ready(ads129x_spi))
    {
        LOG_ERR("SPI device %s is not ready\n", ads129x_spi->name);
        return;
    }

    // initialize the data ready, reset, and start pins
#ifdef DRDY_NODE
    if (!device_is_ready(drdy_spec.port))
    {
        LOG_ERR("Error: %s device is not ready\n", drdy_spec.port->name);
        return;
    }

    ret = gpio_pin_configure_dt(&drdy_spec, GPIO_INPUT);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure pin %d (DRDY)\n", ret, drdy_spec.pin);
        return;
    }
#endif

#ifdef RESET_NODE
    if (!device_is_ready(reset_spec.port))
    {
        LOG_ERR("Error: %s device is not ready\n", reset_spec.port->name);
        return;
    }

    ret = gpio_pin_configure_dt(&reset_spec, GPIO_OUTPUT);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure pin %d (RESET)\n", ret, reset_spec.pin);
        return;
    }
#endif

#ifdef START_NODE
    if (!device_is_ready(start_spec.port))
    {
        LOG_ERR("Error: %s device is not ready\n", start_spec.port->name);
        return;
    }

    ret = gpio_pin_configure_dt(&start_spec, GPIO_OUTPUT);
    if (ret != 0)
    {
        LOG_ERR("Error %d: failed to configure pin %d (START)\n", ret, start_spec.pin);
        return;
    }

#endif
}

void ads129x_setup(void)
{
    // Wait for 18 tCLKs AKA 30*18 microseconds
    ads129x_init();
    gpio_pin_set_dt(&reset_spec, 1);
    k_usleep(ADS_CLK_PERIOD_US * 18);
    gpio_pin_set_dt(&reset_spec, 0);
    k_usleep(ADS_CLK_PERIOD_US * 18);
    gpio_pin_set_dt(&reset_spec, 1);
    k_usleep(ADS_CLK_PERIOD_US * 18);

    // device wakes up in RDATAC mode, so send stop signal
    ads129x_sdatac();
    
    // enable 32kHz sample-rate
    int16_t data_rate = ads129x_get_reg_DR_from_speed(32000);
    uint8_t reg_val = 0; 
    WRITE_BIT(reg_val, ADS129X_BIT_HR, 1);
    WRITE_BIT(reg_val, ADS129X_BIT_DR0, data_rate);
    ads129x_safe_write_register(ADS129X_REG_CONFIG1, reg_val);

    // enable internal reference
    uint8_t enable_internal_reference = (1 << ADS129X_BIT_PD_REFBUF) | (1 << 6);
    ads129x_safe_write_register(ADS129X_REG_CONFIG3, enable_internal_reference);

    uint8_t dev_id = 0;
    ads129x_get_device_id(&dev_id);
    LOG_INF("Device ID: %d", dev_id);

    // setup channels
    ads129x_configChannel(1, false, ADS129X_GAIN_12X, ADS129X_MUX_NORMAL);
    ads129x_configChannel(2, false, ADS129X_GAIN_12X, ADS129X_MUX_NORMAL);
    for (int i = 3; i <= 8; i++)
    {
        ads129x_configChannel(i, false, ADS129X_GAIN_1X, ADS129X_MUX_SHORT);
    }

    gpio_pin_set_dt(&start_spec, 0);
}

uint32_t ads129x_get_claim_data(uint8_t **load_data, uint32_t size)
{
    /**
     * wait for data be ready to read, there is no need
     * to keep asking for result when there is nothing in buffer
     */
    k_sem_take(&ads129x_ring_buffer_rdy, K_FOREVER);

    /**
     * ensure that sufficient data is present
     */
    if (ring_buf_size_get(&ads129x_ring_buffer) < size ||
        k_mutex_lock(&ads129x_ring_buffer_mutex, K_MSEC(100)) != 0)
    {
        return 0;
    }

    return ring_buf_get_claim(&ads129x_ring_buffer, load_data, size);
}

uint32_t ads129x_get_data(uint8_t *load_data, uint32_t size)
{
    /**
     * wait for data be ready to read, there is no need
     * to keep asking for result when there is nothing in buffer
     */
    k_sem_take(&ads129x_ring_buffer_rdy, K_FOREVER);

    /**
     * ensure that sufficient data is present
     */
    if (ring_buf_size_get(&ads129x_ring_buffer) < size ||
        k_mutex_lock(&ads129x_ring_buffer_mutex, K_MSEC(100)) != 0)
    {
        return 0;
    }

    size = ring_buf_get(&ads129x_ring_buffer, load_data, size);
    k_mutex_unlock(&ads129x_ring_buffer_mutex);
    return size;
}

int8_t ads129x_reset_data(void)
{
    if (k_mutex_lock(&ads129x_ring_buffer_mutex, K_MSEC(100)) != 0)
    {
        LOG_ERR("ADS129x: Buffer reset was unsuccessful");
        return -1;
    }

    ring_buf_reset(&ads129x_ring_buffer);
    k_sem_reset(&ads129x_ring_buffer_rdy);
    k_mutex_unlock(&ads129x_ring_buffer_mutex);
    return 0;
}

void ads129x_finish_data(uint8_t *load_data, uint32_t size)
{
    /* Indicate amount of valid data. rx_size can be equal or less than size. */
    ring_buf_get_finish(&ads129x_ring_buffer, size);
    k_mutex_unlock(&ads129x_ring_buffer_mutex);
}

void ads129x_main_thread(void)
{
    ads129x_setup();
    uint8_t* buffer_tracker = NULL;
    struct spi_buf ads129x_rx_bufs[] = {{.buf = NULL, .len = ADS129X_SPI_PACKAGE_SIZE}};
    struct spi_buf_set ads129x_rx = {.buffers = ads129x_rx_bufs, .count = 1};
    uint16_t size = 0;
    uint32_t lead1 = 0;
    uint32_t lead2 = 0;

    for (;;)
    {
        /*
         * Wait for semaphore from ISR; if acquired, do related work, then
         * go to next loop iteration (the semaphore might have been given
         * again); else, make the CPU idle.
         */

        if (k_sem_take(&ads129x_new_data, K_USEC(100)) == 0 && ring_buf_capacity_get(&ads129x_ring_buffer) >= ADS129x_DATA_BUFFER_SIZE)
        {
            k_mutex_lock(&ads129x_ring_buffer_mutex, K_MSEC(100));

            /* Allocate buffer within a ring buffer memory. */
            size = ring_buf_put_claim(&ads129x_ring_buffer, &buffer_tracker, ADS129x_DATA_BUFFER_SIZE);
            ads129x_rx_bufs[0].buf = buffer_tracker;

            /* do processing */
            /* NOTE: Work directly on a ring buffer memory */
            int ret = spi_read(ads129x_spi, &ads129x_spi_cfg, &ads129x_rx);
            if (!ret)
            {
                /* add missing leads */
                lead1 = ads129x_get_leadI(buffer_tracker);
                lead2 = ads129x_get_leadII(buffer_tracker);
                conv_u24_to_raw(ads129x_get_leadIII(lead1, lead2), buffer_tracker, ADS129x_LEAD3_OFFSET);
                conv_u24_to_raw(ads129x_get_aVR(lead1, lead2), buffer_tracker, ADS129x_AVR_OFFSET);
                conv_u24_to_raw(ads129x_get_aVL(lead1, lead2), buffer_tracker, ADS129x_AVL_OFFSET);
                conv_u24_to_raw(ads129x_get_aVF(lead1, lead2), buffer_tracker, ADS129x_AVF_OFFSET);

                ads129x_dump_data(buffer_tracker);
            }

            /* Indicate amount of valid data. rx_size can be equal or less than size. */
            ring_buf_put_finish(&ads129x_ring_buffer, size);

            k_mutex_unlock(&ads129x_ring_buffer_mutex);
            k_sem_give(&ads129x_ring_buffer_rdy);
        }
        else
        {
            /* put CPU to sleep to save power */
            k_usleep(100);
        }
    }
}

K_THREAD_DEFINE(ads129x_th, STACKSIZE, ads129x_main_thread, NULL, NULL, NULL, PRIORITY, K_ESSENTIAL, 0);

// ######
// UTILS

int16_t ads129x_get_reg_DR_from_speed(uint16_t expression) {
    switch (expression)
    {
        case 32000: return 0b000;
        case 16000: return 0b001;
        case 8000: return 0b010;
        case 4000: return 0b011;
        case 2000: return 0b100;
        case 1000: return 0b101;
        case 500: return 0b110;
        default: return -1;
    }
}

#endif