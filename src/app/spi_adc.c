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
 * https://github.com/ferdinandkeil/ADS129X
 * Modified by Ferdinand Keil
 */

#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
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



#ifdef CONFIG_SPI

// data rate
uint16_t data_rate = 2000;

/* size of stack area used by each thread */
#define STACKSIZE 4096
/* scheduling priority used by each thread */
#define PRIORITY 7
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

#define BYTE_TO_BINARY_PATTERN "0b%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
    (byte & 0x80 ? '1' : '0'),     \
        (byte & 0x40 ? '1' : '0'), \
        (byte & 0x20 ? '1' : '0'), \
        (byte & 0x10 ? '1' : '0'), \
        (byte & 0x08 ? '1' : '0'), \
        (byte & 0x04 ? '1' : '0'), \
        (byte & 0x02 ? '1' : '0'), \
        (byte & 0x01 ? '1' : '0')

// ###########
// # BUFFER
// ###########

// stores n numbers of spi data packet
// ring buffer size should be bigger then 251 data len
// here is set to be 3 times bigger
#define ADS129X_RING_BUFFER_PACKET ((uint8_t)(251 / ADS129x_DATA_BUFFER_SIZE))
#define ADS129X_RING_BUFFER_SIZE (ADS129X_RING_BUFFER_PACKET * ADS129x_DATA_BUFFER_SIZE * 3)

// handle simple data requests
K_MUTEX_DEFINE(ads129x_ring_buffer_mutex);
RING_BUF_DECLARE(ads129x_ring_buffer, ADS129X_RING_BUFFER_SIZE * 12);

// ##########
// # DRIVER
// ##########

const struct device *ads129x_spi = DEVICE_DT_GET(SPI_NODE);
#ifdef CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP
const struct spi_cs_control ads129x_cs_ctrl = {
    .gpio.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
    .delay = ADS129X_SPI_CLOCK_DELAY,
    .gpio.pin = 4,
    .gpio.dt_flags = GPIO_ACTIVE_LOW};
#else
const struct spi_cs_control ads129x_cs_ctrl = {
    .gpio.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .delay = ADS129X_SPI_CLOCK_DELAY,
    .gpio.pin = 31,
    .gpio.dt_flags = GPIO_ACTIVE_LOW};
#endif
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

    if (cmd & ADS129X_CMD_RREG)
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
    else if (cmd == ADS129X_CMD_RDATA)
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
 * Reset Registers to Default Values.
 */
void ads129x_reset_pin(void)
{
    LOG_DBG("CMD: Reset via pins");
    gpio_pin_set_dt(&reset_spec, 1);
    k_usleep(ADS_CLK_PERIOD_US * 18);
    gpio_pin_set_dt(&reset_spec, 0);
    k_usleep(ADS_CLK_PERIOD_US * 18);
    gpio_pin_set_dt(&reset_spec, 1);
    k_usleep(ADS_CLK_PERIOD_US * 18);
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

    LOG_INF("WR: %02X (data=" BYTE_TO_BINARY_PATTERN ")", _address & 0x1F, BYTE_TO_BINARY(_value));
    if (tmp != _value)
    {
        LOG_ERR("RR: %02X (data=" BYTE_TO_BINARY_PATTERN ")", _address & 0x1F, BYTE_TO_BINARY(tmp));
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
uint8_t ads129x_get_device_id()
{
    uint8_t dev_id = 0;
    ads129x_read_registers(ADS129X_REG_ID, 1, &dev_id);
    return dev_id;
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
    ads129x_safe_write_register(ADS129X_REG_CH1SET + (_channel - 1), value);
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
    LOG_INF("Setup ecg device");

    // Wait for 18 tCLKs AKA 30*18 microseconds
    ads129x_init();
    ads129x_reset_pin();

    // device wakes up in RDATAC mode, so send stop signal
    ads129x_sdatac();

    /*
     * enable 2kHz sample-rate
     * 4kHz is max which can be handle by BLE 5.2
     */
    ads129x_set_data_rate(data_rate);

    /*
     * enable internal reference
     * this free bit is a previous reserved value
     */
    ads129x_safe_write_register(ADS129X_REG_CONFIG3, (1 << ADS129X_BIT_PD_REFBUF) + (1 << 6));

    // setup channels
    ads129x_configChannel(1, false, ADS129X_GAIN_12X, ADS129X_MUX_NORMAL);
    ads129x_configChannel(2, false, ADS129X_GAIN_12X, ADS129X_MUX_NORMAL);
    for (int i = 3; i <= 8; i++)
    {
        ads129x_configChannel(i, false, ADS129X_GAIN_1X, ADS129X_MUX_SHORT);
    }

    gpio_pin_set_dt(&start_spec, 0);

    LOG_INF("Device ID: %d", ads129x_get_device_id());
    ads129x_dump_regs();
}

// #########
// # UTILS
// #########

// drive adc config
bool ecg_status = false;

int16_t ads129x_get_reg_DR_from_speed(uint16_t expression)
{
    switch (expression)
    {
    case 32000:
        return 0b000;
    case 16000:
        return 0b001;
    case 8000:
        return 0b010;
    case 4000:
        return 0b011;
    case 2000:
        return 0b100;
    case 1000:
        return 0b101;
    case 500:
        return 0b110;
    default:
        return -1;
    }
}

void ads129x_data_enable()
{
    ads129x_start();
    k_msleep(1);
    ads129x_rdatac();
    LOG_INF("Data transfer enabled");
    ecg_status = true;

    ads129x_reset_data();
}

void ads129x_data_disable()
{
    ads129x_sdatac();
    k_msleep(1);
    ads129x_stop();
    LOG_INF("Data transfer disabled");
    ecg_status = false;
}

bool ads129x_get_status()
{
    return ecg_status;
}

int16_t ads129x_set_data_rate(uint16_t _data_rate)
{
    int16_t data_rate_code = ads129x_get_reg_DR_from_speed(_data_rate);

    if (data_rate_code < 0)
    {
        LOG_ERR("Unknown speed parameter value, please check help");
        return -EINVAL;
    }

    LOG_INF("Setting data rate to: %" PRIu16, _data_rate);
    uint8_t reg_val = 0;
    ads129x_read_registers(ADS129X_REG_CONFIG1, 1, &reg_val);

#define DR012_MASK 0b111

    /* clear bits for DR reg */
    reg_val &= ~(DR012_MASK);
    reg_val |= data_rate_code;

    reg_val |= 1 << ADS129X_BIT_HR;
    ads129x_safe_write_register(ADS129X_REG_CONFIG1, reg_val);


    data_rate = _data_rate;
    return 0;
}

uint16_t ads129x_get_data_rate() {
    return data_rate;
}

void ads129x_dump_regs()
{
    uint8_t reg_val = 0;

    struct _dump_regs
    {
        char *name;
        uint8_t address;
    };

    struct _dump_regs regs[4] = {
        {
            .name = "Config1",
            .address = ADS129X_REG_CONFIG1,
        },
        {
            .name = "Config2",
            .address = ADS129X_REG_CONFIG2,
        },
        {
            .name = "Config3",
            .address = ADS129X_REG_CONFIG3,
        },
        {
            .name = "Config4",
            .address = ADS129X_REG_CONFIG4,
        }};
    for (int i = 0; i < 4; i++)
    {
        ads129x_read_registers(regs[i].address, 1, &reg_val);
        LOG_INF("%s: " BYTE_TO_BINARY_PATTERN, regs[i].name, BYTE_TO_BINARY(reg_val));
    }
}

// ###########
// # THREAD
// ###########

// handle data stream
K_PIPE_DEFINE(ads129x_pipe, ADS129X_RING_BUFFER_SIZE * 12, 4);

static pipe_packet_u tx_data;

/*
 * in this spi buffer not whole data is stored
 * it only uses to map spi data map
 */
static struct spi_buf ads129x_rx_bufs[] = {{.buf = tx_data.packet.leads._buffer, .len = ADS129X_SPI_PACKAGE_SIZE}};
static struct spi_buf_set ads129x_rx = {.buffers = ads129x_rx_bufs, .count = 1};

/* track status of buffers */
static size_t total_size = ADS129x_DATA_BUFFER_SIZE;
static size_t bytes_written = 0;

/* tmp vars*/
static int32_t lead1 = 0;
static int32_t lead2 = 0;


int64_t timestamp = 0;

int8_t ads129x_reset_data(void)
{
    timestamp = k_uptime_get();

    ring_buf_reset(&ads129x_ring_buffer);
    k_pipe_flush(&ads129x_pipe);
    return 0;
}


/**
 * Read data by command; supports multiple read back.
 */
void ads129x_read_data_continuous(void)
{

    if (k_mutex_lock(&ads129x_ring_buffer_mutex, K_MSEC(100)) != 0) {
        return;
    }

    if (ring_buf_space_get(&ads129x_ring_buffer) < ADS129x_DATA_BUFFER_SIZE)
    {
        k_mutex_unlock(&ads129x_ring_buffer_mutex);
        return;
    }

    uint8_t *data = NULL;

    /* Allocate buffer within a ring buffer memory. */
    uint16_t size = ring_buf_put_claim(&ads129x_ring_buffer, &data, ADS129x_DATA_BUFFER_SIZE);

    /* add timestamp */
    uint32_t tmp_timestamp = (uint32_t)(k_uptime_get() - timestamp);
    data = conv_u24_to_raw(tmp_timestamp, data, 0);

    /* do processing */
    /* NOTE: Work directly on a ring buffer memory */
    int ret = spi_read(ads129x_spi, &ads129x_spi_cfg, &ads129x_rx);
    if (!ret)
    {
        /* add missing leads */
        /* add missing leads */
        lead1 = conv_u24_to_i32(conv_raw_to_u24(tx_data.packet.leads._buffer, ADS129x_LEAD1_OFFSET));
        lead2 = conv_u24_to_i32(conv_raw_to_u24(tx_data.packet.leads._buffer, ADS129x_LEAD2_OFFSET));
        conv_u24_to_raw(ads129x_get_leadIII(lead1, lead2), tx_data.packet.leads._buffer, ADS129x_LEAD3_OFFSET);
        conv_u24_to_raw(ads129x_get_aVR(lead1, lead2), tx_data.packet.leads._buffer, ADS129x_AVR_OFFSET);
        conv_u24_to_raw(ads129x_get_aVL(lead1, lead2), tx_data.packet.leads._buffer, ADS129x_AVL_OFFSET);
        conv_u24_to_raw(ads129x_get_aVF(lead1, lead2), tx_data.packet.leads._buffer, ADS129x_AVF_OFFSET);

    }
    /* Indicate amount of valid data. rx_size can be equal or less than size. */
    ring_buf_put_finish(&ads129x_ring_buffer, size);

    k_mutex_unlock(&ads129x_ring_buffer_mutex);
}

/**
 * Read data by command; supports multiple read back.
 */
uint32_t ads129x_write_data_continuous(uint8_t **buffer, uint32_t size)
{

    if (k_mutex_lock(&ads129x_ring_buffer_mutex, K_SECONDS(60)) != 0) {
        return 0;
    }

    if (ring_buf_size_get(&ads129x_ring_buffer) < size)
    {
        k_mutex_unlock(&ads129x_ring_buffer_mutex);
        return 0;
    }

    /* Allocate buffer within a ring buffer memory. */
    return ring_buf_get_claim(&ads129x_ring_buffer, buffer, size);
}

void ads129x_write_data_continuous_fin(uint32_t size){
    /* do processing */
    /* NOTE: Work directly on a ring buffer memory */
    /* Indicate amount of valid data. rx_size can be equal or less than size. */
    ring_buf_put_finish(&ads129x_ring_buffer, size);

    k_mutex_unlock(&ads129x_ring_buffer_mutex);
}

uint32_t ads129x_get_data(uint8_t *load_data, uint32_t size)
{
    int rc = 0;
    uint32_t total = size;
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

        rc = k_pipe_get(&ads129x_pipe, load_data, size, &bytes_read, min_size, K_SECONDS(10));

        if (rc == -EINVAL)
        {
            LOG_ERR("Bad input data: size=%d, min_size=%d, read=%d", size, min_size, bytes_read);
            break;
        }
        else if ((rc < 0) || (bytes_read < min_size))
        {
            LOG_DBG("Waiting period timed out; between zero and min_xfer minus one data bytes were read. %d", rc);
            continue;
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
    /* add timestamp */
    tx_data.packet.timestamp = k_uptime_get() - timestamp;
    conv_u24_to_raw(tx_data.packet.timestamp, tx_data.buffer, 0);

    /* do processing */
    /* NOTE: Work directly on a ring buffer memory */
    int ret = spi_read(ads129x_spi, &ads129x_spi_cfg, &ads129x_rx);
    if (!ret)
    {
        /* add missing leads */
        lead1 = conv_u24_to_i32(conv_raw_to_u24(tx_data.packet.leads._buffer, ADS129x_LEAD1_OFFSET));
        lead2 = conv_u24_to_i32(conv_raw_to_u24(tx_data.packet.leads._buffer, ADS129x_LEAD2_OFFSET));
        conv_u24_to_raw(ads129x_get_leadIII(lead1, lead2), tx_data.packet.leads._buffer, ADS129x_LEAD3_OFFSET);
        conv_u24_to_raw(ads129x_get_aVR(lead1, lead2), tx_data.packet.leads._buffer, ADS129x_AVR_OFFSET);
        conv_u24_to_raw(ads129x_get_aVL(lead1, lead2), tx_data.packet.leads._buffer, ADS129x_AVL_OFFSET);
        conv_u24_to_raw(ads129x_get_aVF(lead1, lead2), tx_data.packet.leads._buffer, ADS129x_AVF_OFFSET);

        //ads129x_dump_data(tx_data.packet.leads._buffer);
    }

    /* send data to the consumers */
    k_pipe_put(&ads129x_pipe, &tx_data.buffer, total_size, &bytes_written, sizeof(pipe_packet_u), K_NO_WAIT);
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

K_THREAD_DEFINE(thread_ads129x, STACKSIZE, ads129x_th, NULL, NULL, NULL, PRIORITY, K_ESSENTIAL, 0);

#endif