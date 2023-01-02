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
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include <cmd.h>
#include <performance_test.h>
#include <bt_test.h>

/**
 * WARN: ADS129x is using BIG_ENDIAN
 * Where most of arms is using a little_endian
 */
#include <spi_adc.h>
#include <app_utils.h>

#ifdef CONFIG_SPI

K_SEM_DEFINE(ads129x_new_data, 0, 100);

LOG_MODULE_REGISTER(ecg, LOG_LEVEL_INF);
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

// ##########
// # DRIVER
// ##########

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
ads129x_config_t ads129x_config = {
    .data_rate = 2000,
    .bytes_to_send = 0,
    .print_data = false,
    .ads129x_spi = DEVICE_DT_GET(SPI_NODE),
    .ads129x_spi_cfg = {
        .frequency = ADS129X_SPI_CLOCK_SPEED,
        .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
        .cs = &ads129x_cs_ctrl},
    .packets_dropped = 0,

};

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

uint32_t set_bytes_to_send(uint32_t _bytes_to_send)
{
    /*
     * we always waiting for data to match whole buffer
     * so data requested should also match it
     * otherwise we should ignore it
     */
    uint16_t remainder = _bytes_to_send % ADS129x_DATA_BUFFER_SIZE;
    if (remainder)
    {
        ads129x_config.bytes_to_send = ((_bytes_to_send / ADS129x_DATA_BUFFER_SIZE) + 1) * ADS129x_DATA_BUFFER_SIZE;
    }
    else
    {
        ads129x_config.bytes_to_send = _bytes_to_send;
    }

    return ads129x_config.bytes_to_send;
}

int ads129x_access(const struct device *_spi,
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
        return spi_transceive(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, &tx, &rx);
    }
    else if (cmd & ADS129X_CMD_WREG)
    {
        tx.count = 3;
        return spi_write(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, &tx);
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
        return spi_write(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, &tx);
    }
}

// System Commands

/**
 * Exit Standby Mode.
 */
void ads129x_wakeup(void)
{
    LOG_DBG("CMD: Wakeup");
    ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, ADS129X_CMD_WAKEUP, NULL, 0);
}

/**
 * Enter Standby Mode.
 */
void ads129x_standby(void)
{
    LOG_DBG("CMD: Standby");
    ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, ADS129X_CMD_STANDBY, NULL, 0);
}

/**
 * Reset Registers to Default Values.
 */
void ads129x_reset(void)
{
    LOG_DBG("CMD: Reset");
    ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, ADS129X_CMD_RESET, NULL, 0);
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
    ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, ADS129X_CMD_START, NULL, 0);
    ads129x_drdy_init_callback();
}

/**
 * Stop conversion.
 */
void ads129x_stop()
{
    LOG_DBG("CMD: Stop");
    ads129x_drdy_callback_deinit();
    ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, ADS129X_CMD_STOP, NULL, 0);
}

/**
 * Enable Read Data Continuous mode (default).
 */
void ads129x_rdatac(void)
{
    LOG_DBG("CMD: Read Data Continuous");
    ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, ADS129X_CMD_RDATAC, NULL, 0);
}

/**
 * Stop Read Data Continuously mode.
 */
void ads129x_sdatac(void)
{
    LOG_DBG("CMD: Stop Read Data Continuous");
    ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, ADS129X_CMD_SDATAC, NULL, 0);
}

/**
 * Locks spi and keeps cs low for purpose of sending multiple commands
 */
static void ads129x_lock_spi(void)
{
    WRITE_BIT_VAL(ads129x_config.ads129x_spi_cfg.operation, (SPI_LOCK_ON | SPI_HOLD_ON_CS), 1);
}

/**
 * Unlocks spi and release cs
 */
static void ads129x_unlock_spi(void)
{
    spi_release(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg);
    WRITE_BIT_VAL(ads129x_config.ads129x_spi_cfg.operation, (SPI_LOCK_ON | SPI_HOLD_ON_CS), 0);
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
    int ret = ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, opcode1, _value, _n);
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
    return ads129x_access(ads129x_config.ads129x_spi, &ads129x_config.ads129x_spi_cfg, opcode1, _value, _n);
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
    ads129x_config.print_data = _print;
}

void ads129x_dump_data(uint8_t *input_data)
{
    int32_t data[ADS129X_DATA_NUM];
    char print_buf[ADS129X_DATA_NUM * 10 + 1];

    if (ads129x_config.print_data)
    {
        uint8_t print_buf_pos = 0;
        memset(data, 0, sizeof(data));

        // data[0] = conv_u24_to_i32(conv_raw_to_u24(input_data, 0));
        // print_buf_pos += snprintk(print_buf, sizeof(print_buf), "%08x ", data[0]);

        for (int i = 1; i < ADS129X_DATA_NUM; i++)
        {
            data[i] = conv_u24_to_i32(conv_raw_to_u24(input_data, i * 3));
            print_buf_pos += snprintk(print_buf + print_buf_pos, sizeof(print_buf) - print_buf_pos, "%9" PRIi32 " ", (data[i] * 300) / 1000);
        }
        LOG_INF("%s", print_buf);
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
    if (!device_is_ready(ads129x_config.ads129x_spi))
    {
        LOG_ERR("SPI device %s is not ready\n", ads129x_config.ads129x_spi->name);
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
    ads129x_set_data_rate(ads129x_config.data_rate);

    /*
     * enable internal reference
     * this free bit is a previous reserved value
     */
    ads129x_safe_write_register(ADS129X_REG_CONFIG3, (1 << ADS129X_BIT_PD_REFBUF) + (1 << 6));

    ads129x_enable_hrm_signal();

    ads129x_wct();
    gpio_pin_set_dt(&start_spec, 0);

    LOG_INF("Device ID: %d", ads129x_get_device_id());
    ads129x_dump_regs();
}

void ads129x_enable_test_signal()
{
    // test mode
    ads129x_safe_write_register(ADS129X_REG_CONFIG2, ADS129X_TEST_FREQ_2HZ | 1 << ADS129X_BIT_TEST_AMP | 1 << ADS129X_BIT_INT_TEST);
    for (int i = 1; i <= 8; i++)
    {
        ads129x_configChannel(i, false, ADS129X_GAIN_1X, ADS129X_MUX_TEST);
    }
}

void ads129x_enable_hrm_signal()
{
    // test mode
    ads129x_safe_write_register(ADS129X_REG_CONFIG2, 0);

    for (int i = 1; i <= 8; i++)
    {
        ads129x_configChannel(i, false, ADS129X_GAIN_6X, ADS129X_MUX_NORMAL);
    }
}

void ads129x_enable_external_test()
{
    // test mode
    ads129x_safe_write_register(ADS129X_REG_CONFIG2, 0);

    for (int i = 1; i <= 8; i++)
    {
        ads129x_configChannel(i, false, ADS129X_GAIN_2X, ADS129X_MUX_NORMAL);
    }
}

void ads129x_enable_supply_voltage_test()
{
    // test mode
    ads129x_safe_write_register(ADS129X_REG_CONFIG2,  ADS129X_TEST_FREQ_DC << ADS129X_BIT_TEST_AMP | 1 << ADS129X_BIT_INT_TEST);
    for (int i = 1; i <= 8; i++)
    {
        ads129x_configChannel(i, false, ADS129X_GAIN_1X, ADS129X_MUX_MVDD);
    }
}

void ads129x_wct()
{
    // 100 = Channel 3 positive input connected to WCTA amplifier
    ads129x_safe_write_register(ADS129X_REG_WCT1, 1 << ADS129X_BIT_PD_WCTA | 1 << ADS129X_BIT_WCTA2);

    ads129x_safe_write_register(
        //    011 = Channel 2 negative input connected to WCTB amplifier
        ADS129X_REG_WCT1,
        (1 << ADS129X_BIT_PD_WCTB | 1 << ADS129X_BIT_WCTB1 | 1 << ADS129X_BIT_WCTB0) |
        // 010 = Channel 2 positive input connected to WCTB amplifier
        (1 << ADS129X_BIT_PD_WCTC | 1 << ADS129X_BIT_WCTC1));
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
}

void ads129x_data_disable()
{
    ads129x_sdatac();
    k_msleep(1);
    ads129x_stop();
    LOG_INF("Data transfer disabled");
    ecg_status = false;

    if (ads129x_config.packets_dropped)
    {
        LOG_ERR("SPI dropped %" PRIu32 " packets of %d size", ads129x_config.packets_dropped, ADS129x_DATA_BUFFER_SIZE);
    }
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

    ads129x_config.data_rate = _data_rate;
    return 0;
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

void ads129x_load_augmented_leads(uint8_t *buffer)
{
    /* tmp vars*/
    static int32_t lead1 = 0;
    static int32_t lead2 = 0;

    /* add missing leads */
    lead1 = conv_u24_to_i32(conv_raw_to_u24(buffer, ADS129x_LEAD1_OFFSET));
    lead2 = conv_u24_to_i32(conv_raw_to_u24(buffer, ADS129x_LEAD2_OFFSET));
    conv_u24_to_raw(ads129x_get_leadIII(lead1, lead2), buffer, ADS129x_LEAD3_OFFSET);
    conv_u24_to_raw(ads129x_get_aVR(lead1, lead2), buffer, ADS129x_AVR_OFFSET);
    conv_u24_to_raw(ads129x_get_aVL(lead1, lead2), buffer, ADS129x_AVL_OFFSET);
    conv_u24_to_raw(ads129x_get_aVF(lead1, lead2), buffer, ADS129x_AVF_OFFSET);
}

#else

ads129x_config_t ads129x_config = {
    .data_rate = 2000,
    .bytes_to_send = 0,
    .print_data = false,
    .packets_dropped = 0,
};

#endif

uint16_t ads129x_get_data_rate()
{
    return ads129x_config.data_rate;
}
