
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

#ifndef ____ADS129X__
#define ____ADS129X__

// SPI Command Definition Byte Assignments (Datasheet, pg. 35)
#define ADS129X_CMD_WAKEUP 0x02  // Wake-up from standby mode
#define ADS129X_CMD_STANDBY 0x04 // Enter Standby mode
#define ADS129X_CMD_RESET 0x06   // Reset the device
#define ADS129X_CMD_START 0x08   // Start and restart (synchronize) conversions
#define ADS129X_CMD_STOP 0x0A    // Stop conversion
#define ADS129X_CMD_RDATAC 0x10  // Enable Read Data Continuous mode (default mode at power-up)
#define ADS129X_CMD_SDATAC 0x11  // Stop Read Data Continuous mode
#define ADS129X_CMD_RDATA 0x12   // Read data by command; supports multiple read back
#define ADS129X_CMD_RREG 0x20    // (also = 00100000) is the first opcode that the address must be added to for RREG communication
#define ADS129X_CMD_WREG 0x40    // 01000000 in binary (Datasheet, pg. 35)

// Register Addresses
#define ADS129X_REG_ID 0x00         // ID Control Register
#define ADS129X_REG_CONFIG1 0x01    // Configuration Register 1
#define ADS129X_REG_CONFIG2 0x02    // Configuration Register 2
#define ADS129X_REG_CONFIG3 0x03    // Configuration Register 3
#define ADS129X_REG_LOFF 0x04       // Lead-Off Control Register
#define ADS129X_REG_CH1SET 0x05     // Individual Channel Settings 1-8
#define ADS129X_REG_CH2SET 0x06     // ---
#define ADS129X_REG_CH3SET 0x07     // ---
#define ADS129X_REG_CH4SET 0x08     // ---
#define ADS129X_REG_CH5SET 0x09     // ---
#define ADS129X_REG_CH6SET 0x0A     // ---
#define ADS129X_REG_CH7SET 0x0B     // ---
#define ADS129X_REG_CH8SET 0x0C     // ---
#define ADS129X_REG_RLD_SENSP 0x0D  // Right Leg Drive, positive side
#define ADS129X_REG_RLD_SENSN 0x0E  // Right Leg Drive, negative side
#define ADS129X_REG_LOFF_SENSP 0x0F // Lead-Off Detection, positive side
#define ADS129X_REG_LOFF_SENSN 0x10 // Lead-Off Detection, negative side
#define ADS129X_REG_LOFF_FLIP 0x11  // Lead-Off Detection, current direction
#define ADS129X_REG_LOFF_STATP 0x12 // Electrode Status, positive (read-only)
#define ADS129X_REG_LOFF_STATN 0x13 // Electrode Status, negative (read-only)
#define ADS129X_REG_GPIO 0x14       // General-Purpose I/O Register
#define ADS129X_REG_PACE 0x15       // PACE Detect Register
#define ADS129X_REG_RESP 0x16       // Respiration Control Register
#define ADS129X_REG_CONFIG4 0x17    // Configuration Register 4
#define ADS129X_REG_WCT1 0x18       // Wilson Central Terminal and Augmented Lead Control Register
#define ADS129X_REG_WCT2 0x19       // Wilson Central Terminal Control Register

// IDs
#define ADS129X_ID_ADS1294 0x90
#define ADS129X_ID_ADS1296 0x91
#define ADS129X_ID_ADS1298 0x92
#define ADS129X_ID_ADS1294R 0xD0
#define ADS129X_ID_ADS1296R 0xD1
#define ADS129X_ID_ADS1298R 0xD2

// Configuration Register 1
#define ADS129X_BIT_HR 0x7
#define ADS129X_BIT_DAISY_EN 0x6
#define ADS129X_BIT_CLK_EN 0x5
// always 0
// always 0
#define ADS129X_BIT_DR2 0x2
#define ADS129X_BIT_DR1 0x1
#define ADS129X_BIT_DR0 0x0

// Configuration Register 2
// always 0
// always 0
#define ADS129X_BIT_WCT_CHOP 0x5
#define ADS129X_BIT_INT_TEST 0x4
// always 0
#define ADS129X_BIT_TEST_AMP 0x2
#define ADS129X_BIT_TEST_FREQ1 0x1
#define ADS129X_BIT_TEST_FREQ0 0x0
#define ADS129X_TEST_FREQ_1HZ 0x0
#define ADS129X_TEST_FREQ_2HZ 0x1
#define ADS129X_TEST_FREQ_DC 0x3

// Configuration Register 3
#define ADS129X_BIT_PD_REFBUF 0x7
// always 1
#define ADS129X_BIT_VREF_4V 0x5
#define ADS129X_BIT_RLD_MEAS 0x4
#define ADS129X_BIT_RLDREF_INT 0x3
#define ADS129X_BIT_PD_RLD 0x2
#define ADS129X_BIT_RLD_LOFF_SENS 0x1
#define ADS129X_BIT_RLD_STAT 0x0

// Lead-Off Control Register
#define ADS129X_BIT_COMP_TH2 0x7
#define ADS129X_BIT_COMP_TH1 0x6
#define ADS129X_BIT_COMP_TH0 0x5
#define ADS129X_BIT_VLEAD_OFF_EN 0x4
#define ADS129X_BIT_ILEAD_OFF1 0x3
#define ADS129X_BIT_ILEAD_OFF0 0x2
#define ADS129X_BIT_FLEAD_OFF1 0x1
#define ADS129X_BIT_FLEAD_OFF0 0x0

// Individual Channel Settings
#define ADS129X_BIT_PD 0x7
#define ADS129X_BIT_GAIN2 0x6
#define ADS129X_BIT_GAIN1 0x5
#define ADS129X_BIT_GAIN0 0x4
// always 0
#define ADS129X_BIT_MUX2 0x2
#define ADS129X_BIT_MUX1 0x1
#define ADS129X_BIT_MUX0 0x0

// Channel Select
#define ADS129X_BIT_CH8 0x7
#define ADS129X_BIT_CH7 0x6
#define ADS129X_BIT_CH6 0x5
#define ADS129X_BIT_CH5 0x4
#define ADS129X_BIT_CH4 0x3
#define ADS129X_BIT_CH3 0x2
#define ADS129X_BIT_CH2 0x1
#define ADS129X_BIT_CH1 0x0

// General-Purpose I/O Register
#define ADS129X_BIT_GPIOD4 0x7
#define ADS129X_BIT_GPIOD3 0x6
#define ADS129X_BIT_GPIOD2 0x5
#define ADS129X_BIT_GPIOD1 0x4
#define ADS129X_BIT_GPIOC4 0x3
#define ADS129X_BIT_GPIOC3 0x2
#define ADS129X_BIT_GPIOC2 0x1
#define ADS129X_BIT_GPIOC1 0x0

// PACE Detect Register
// always 0
// always 0
// always 0
#define ADS129X_BIT_PACEE1 0x4
#define ADS129X_BIT_PACEE0 0x3
#define ADS129X_BIT_PACEO1 0x2
#define ADS129X_BIT_PACEO0 0x1
#define ADS129X_BIT_PD_PACE 0x0

// Respiration Control Register
#define ADS129X_BIT_RESP_DEMOD_EN1 0x7
#define ADS129X_BIT_RESP_MOD_EN1 0x6
// always 1
#define ADS129X_BIT_RESP_PH2 0x4
#define ADS129X_BIT_RESP_PH1 0x3
#define ADS129X_BIT_RESP_PH0 0x2
#define ADS129X_BIT_RESP_CTRL1 0x1
#define ADS129X_BIT_RESP_CTRL0 0x0

// Configuration Register 4
#define ADS129X_BIT_RESP_FREQ2 0x7
#define ADS129X_BIT_RESP_FREQ1 0x6
#define ADS129X_BIT_RESP_FREQ0 0x5
// always 0
#define ADS129X_BIT_SINGLE_SHOT 0x3
#define ADS129X_BIT_WCT_TO_RLD 0x2
#define ADS129X_BIT_PD_LOFF_COMP 0x1
// always 0

// Wilson Central Terminal and Augmented Lead Control Register
#define ADS129X_BIT_aVF_CH6 0x7
#define ADS129X_BIT_aVF_CH5 0x6
#define ADS129X_BIT_aVF_CH7 0x5
#define ADS129X_BIT_aVF_CH4 0x4
#define ADS129X_BIT_PD_WCTA 0x3
#define ADS129X_BIT_WCTA2 0x2
#define ADS129X_BIT_WCTA1 0x1
#define ADS129X_BIT_WCTA0 0x0

// Wilson Central Terminal Control Register
#define ADS129X_BIT_PD_WCTC 0x7
#define ADS129X_BIT_PD_WCTB 0x6
#define ADS129X_BIT_WCTB2 0x5
#define ADS129X_BIT_WCTB1 0x4
#define ADS129X_BIT_WCTB0 0x3
#define ADS129X_BIT_WCTC2 0x2
#define ADS129X_BIT_WCTC1 0x1
#define ADS129X_BIT_WCTC0 0x0

// Gain Configuration
#define ADS129X_GAIN_6X 0x0
#define ADS129X_GAIN_1X 0x1
#define ADS129X_GAIN_2X 0x2
#define ADS129X_GAIN_3X 0x3
#define ADS129X_GAIN_4X 0x4
#define ADS129X_GAIN_8X 0x5
#define ADS129X_GAIN_12X 0x6

// Mux Configuration
#define ADS129X_MUX_NORMAL 0x0   // Normal electrode input (default)
#define ADS129X_MUX_SHORT 0x1    // Input shorted (for offset or noise measurements)
#define ADS129X_MUX_RLD_MEAS 0x2 // Used in conjunction with RLD_MEAS bit for RLD measurements
#define ADS129X_MUX_MVDD 0x3     // MVDD for supply measurement
#define ADS129X_MUX_TEMP 0x4     // Temperature sensor
#define ADS129X_MUX_TEST 0x5     // Test signal
#define ADS129X_MUX_RLD_DRP 0x6  // RLD_DRP (positive electrode is the driver)
#define ADS129X_MUX_RLD_DRN 0x7  // RLD_DRN (negative electrode is the driver)

// Sample-rate Configuration
#define ADS129X_SAMPLERATE_1024 0x6
#define ADS129X_SAMPLERATE_512 0x5
#define ADS129X_SAMPLERATE_256 0x4
#define ADS129X_SAMPLERATE_128 0x3
#define ADS129X_SAMPLERATE_64 0x2
#define ADS129X_SAMPLERATE_32 0x1
#define ADS129X_SAMPLERATE_16 0x0

#define ADS129X_SPI_CLOCK_SPEED 4000000UL
#define ADS129X_SPI_CLOCK_DELAY ((1000000 * 8) / ADS129X_SPI_CLOCK_SPEED) // must send at least 4 tCLK cycles before sending another command (Datasheet, pg. 38)
#define ADS129X_SPI_STATUS_WORD_NUM 1
#define ADS129X_SPI_DATA_PACKAGES_NUM 8
#define ADS129X_SPI_PACKAGE_NUM (ADS129X_SPI_DATA_PACKAGES_NUM + ADS129X_SPI_STATUS_WORD_NUM)
#define ADS129X_SPI_PACKAGE_SIZE (ADS129X_SPI_PACKAGE_NUM * 3)

#define ADS129X_AUGMENTED_LEAD_NUM 4

#define ADS129X_ECG_NUM (ADS129X_AUGMENTED_LEAD_NUM + ADS129X_SPI_DATA_PACKAGES_NUM)
#define ADS129X_ECG_SIZE (ADS129X_ECG_NUM * 3)

#define ADS129x_TIMESTAMP_NUM 1
#define ADS129X_DATA_NUM (ADS129X_SPI_DATA_PACKAGES_NUM + ADS129X_SPI_STATUS_WORD_NUM + ADS129X_AUGMENTED_LEAD_NUM + ADS129x_TIMESTAMP_NUM)
#define ADS129x_DATA_BUFFER_SIZE (ADS129X_DATA_NUM * 3)

typedef struct  __attribute__((__packed__)) _packet_12lead_struct
{
    uint32_t status : 24;
    uint32_t v6_c6 : 24;
    uint32_t lead1 : 24;
    uint32_t lead2 : 24;
    uint32_t v2_c2 : 24;
    uint32_t v3_c3 : 24;
    uint32_t v4_c4 : 24;
    uint32_t v5_c5 : 24;
    uint32_t v1_c1 : 24;

    uint32_t lead3 : 24;
    uint32_t aVR : 24;
    uint32_t aVL : 24;
    uint32_t aVF : 24;

} packet_12lead_t;

typedef union _packet_12lead_u {
    packet_12lead_t _leads;
    uint8_t _buffer[sizeof(packet_12lead_t)];
} packet_12lead_u;

typedef struct  __attribute__((__packed__)) _pipe_packet {
    uint32_t timestamp: 24;
    packet_12lead_u leads;
} pipe_packet_t;

typedef union _pipe_packet_u {
    pipe_packet_t packet;
    uint8_t buffer[sizeof(pipe_packet_t)];
} pipe_packet_u;


#define ADS129x_STATUS_OFFSET   (0 * 3)
#define ADS129x_V6_OFFSET       (1 * 3)
#define ADS129x_LEAD1_OFFSET    (2 * 3)
#define ADS129x_LEAD2_OFFSET    (3 * 3)
#define ADS129x_V2_OFFSET       (4 * 3)
#define ADS129x_V3_OFFSET       (5 * 3)
#define ADS129x_V4_OFFSET       (6 * 3)
#define ADS129x_V5_OFFSET       (7 * 3)
#define ADS129x_V1_OFFSET       (8 * 3)

#define ADS129x_LEAD3_OFFSET    (9 * 3)
#define ADS129x_AVR_OFFSET      (10 * 3)
#define ADS129x_AVL_OFFSET      (11 * 3)
#define ADS129x_AVF_OFFSET      (12 * 3)

typedef union _ads129x_data_packet_t
{
    uint8_t packet_raw[ADS129x_DATA_BUFFER_SIZE];
    packet_12lead_t packet_12lead;
} ads129x_data_packet_t;

uint32_t ads129x_get_data(uint8_t *load_data, uint32_t size);

void ads129x_write_data_continuous_fin(uint32_t size);
uint32_t ads129x_write_data_continuous(uint8_t **buffer, uint32_t size);

int8_t ads129x_reset_data(void);

void ads129x_init(void);
void ads129x_wakeup(void);
void ads129x_standby(void);
void ads129x_start(void);
void ads129x_stop(void);
void ads129x_reset(void);
void ads129x_rdatac(void);
void ads129x_sdatac(void);

void ads129x_read_data(void);
void ads129x_write_data(void);

int ads129x_read_registers(uint8_t _address, uint8_t _n, uint8_t *_value);
int ads129x_write_registers(uint8_t _address, uint8_t _n, uint8_t *_value);

void ads129x_read_data_single(void);
void ads129x_write_data_single(void);

uint8_t ads129x_get_device_id();
void ads129x_setup();
void ads129x_print(bool _print);


int16_t ads129x_get_reg_DR_from_speed(uint16_t speed);
void ads129x_data_enable();
void ads129x_data_disable();
bool ads129x_get_status();
int16_t ads129x_set_data_rate(uint16_t data_rate);
uint16_t ads129x_get_data_rate();

void ads129x_dump_regs();

#endif
