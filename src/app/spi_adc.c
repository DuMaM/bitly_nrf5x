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

#include <kernel.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <drivers/spi.h>
#include <sys/printk.h>

#include <spi_adc.h>

int ADS129X_CS;
long ADS129X_data[9];
bool ADS129X_newData;
void ADS129X_dataReadyISR();

#define RESET_NODE DT_NODELABEL(ads129x_reset)
#define DRDY_NODE DT_NODELABEL(ads129x_drdy)

struct spi_config spi_cfg = {0};
struct gpio_dt_spec drdy_spec = GPIO_DT_SPEC_GET_OR(DRDY_NODE, gpios, {0});
struct gpio_dt_spec reset_spec = GPIO_DT_SPEC_GET_OR(RESET_NODE, gpios, {0});

#if !DT_NODE_HAS_STATUS(DRDY_NODE, okay)
#error "Unsupported board: drdy devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(RESET_NODE, okay)
#error "Unsupported board: drdy devicetree alias is not defined"
#endif

/**
 * Initializes ADS129x library.
 */
void ads129x_init(void) {
    // SPI Setup
    const struct device *spi = DEVICE_DT_GET(DT_NODELABEL(spi2));
    int ret;

    printk("SPI init\n");
    if (!device_is_ready(spi)) {
        printk("SPI device %s is not ready\n", spi->name);
        return;
    }

    spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA;
    spi_cfg.frequency = 4000000U;


    // initialize the data ready and chip select pins
    if (!device_is_ready(drdy_spec.port)) {
        printk("Error: %s device is not ready\n", drdy_spec.port->name);
        return;
    }

    if (!device_is_ready(reset_spec.port)) {
        printk("Error: %s device is not ready\n", reset_spec.port->name);
        return;
    }

    ret = gpio_pin_configure_dt(&drdy_spec, GPIO_PULL_UP);
    if (ret != 0) {
        printk("Error %d: failed to configure pin %d (DRDY)\n", ret, drdy_spec.pin);
        return;
    }

    ret = gpio_pin_configure_dt(&reset_spec, GPIO_OUTPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure pin %d (RESET)\n", ret, reset_spec.pin);
        return;
    }
}

// struct {
//     uint8_t ini_test;
// } config2;


// int spi_adc_init(void);
// int spi_adc_write_to_reg(uint8_t reg, uint8_t value);
// int spi_adc_set_test_signal(int amp, int freq, bool external)
// {
//     if (amp < 0 || amp > 255)
//         return -1;

//     if (amp > 0) {
//         spi_adc_write_to_reg(REG_TEST_AMP, amp);
//     }

//     if (freq < 0 || freq > 255)
//         return -1;

//     if (freq > 0) {
//         spi_adc_write_to_reg(REG_TEST_FREQ, freq);
//     }

//     if (external) {
//         spi_adc_write_to_reg(REG_CONFIG2, MUX_TEST_SIGNAL);
//     }
// }

static int mb85rs64v_access(const struct device *spi,
                struct spi_config *spi_cfg,
                uint8_t cmd, uint16_t addr, void *data, size_t len)
{
    uint8_t access[3];
    struct spi_buf bufs[] = {
        {
            .buf = access,
        },
        {
            .buf = data,
            .len = len
        }
    };
    struct spi_buf_set tx = {
        .buffers = bufs
    };

    access[0] = cmd;

    if (cmd == ADS129X_CMD_WREG || cmd == ADS129X_CMD_RDATA || cmd == ADS129X_CMD_RDATAC || cmd == ADS129X_CMD_RDATAC) {
        access[1] = (addr >> 8) & 0xFF;
        access[2] = addr & 0xFF;

        bufs[0].len = 3;
        tx.count = 2;

        if (cmd != ADS129X_CMD_WREG) {
            struct spi_buf_set rx = {
                .buffers = bufs,
                .count = 2
            };

            return spi_transceive(spi, spi_cfg, &tx, &rx);
        }
    } else {
        tx.count = 1;
    }

    return spi_write(spi, spi_cfg, &tx);
}

//System Commands

/**
//  * Exit Standby Mode.
//  */
// void ADS129X::WAKEUP() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW); //Low to communicate
//     SPI.transfer(ADS129X_CMD_WAKEUP);
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH); //High to end communication
//     delayMicroseconds(2);  //must way at least 4 tCLK cycles before sending another command (Datasheet, pg. 38)
//     SPI.endTransaction();
// }

// /**
//  * Enter Standby Mode.
//  */
// void ADS129X::STANDBY() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW);
//     SPI.transfer(ADS129X_CMD_STANDBY);
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH);
//     SPI.endTransaction();
// }

// /**
//  * Reset Registers to Default Values.
//  */
// void ADS129X::RESET() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW);
//     SPI.transfer(ADS129X_CMD_RESET);
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH);
//     delay(10); //must wait 18 tCLK cycles to execute this command (Datasheet, pg. 38)
//     SPI.endTransaction();
// }

// /**
//  * Start/restart (synchronize) conversions.
//  */
// void ADS129X::START() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW);
//     SPI.transfer(ADS129X_CMD_START);
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH);
//     SPI.endTransaction();
// #ifndef ADS129X_POLLING
//     attachInterrupt(DRDY, ADS129X_dataReadyISR, FALLING);
// #endif
// }

// /**
//  * Stop conversion.
//  */
// void ADS129X::STOP() {
// #ifndef ADS129X_POLLING
//         detachInterrupt(DRDY);
// #endif
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW);
//     SPI.transfer(ADS129X_CMD_STOP);
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH);
//     SPI.endTransaction();
// }

// /**
//  * Enable Read Data Continuous mode (default).
//  */
// void ADS129X::RDATAC() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW);
//     SPI.transfer(ADS129X_CMD_RDATAC);
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH);
//     delayMicroseconds(2); //must way at least 4 tCLK cycles before sending another command (Datasheet, pg. 39)
//     SPI.endTransaction();
// }

// /**
//  * Stop Read Data Continuously mode.
//  */
// void ADS129X::SDATAC() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW);
//     SPI.transfer(ADS129X_CMD_SDATAC); //SDATAC
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH);
//     SPI.endTransaction();
// }

// /**
//  * Read data by command; supports multiple read back.
//  */
// void ADS129X::RDATA() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW);
//     SPI.transfer(ADS129X_CMD_RDATA);
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH);
//     SPI.endTransaction();
// }

// /**
//  * Read register at address _address.
//  * @param  _address register address
//  * @return          value of register
//  */
// byte ADS129X::RREG(byte _address) {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     byte opcode1 = ADS129X_CMD_RREG | (_address & 0x1F); //001rrrrr; _RREG = 00100000 and _address = rrrrr
//     digitalWrite(CS, LOW); //Low to communicate
//     SPI.transfer(opcode1); //RREG
//     SPI.transfer(0x00); //opcode2
//     delayMicroseconds(1);
//     byte data = SPI.transfer(0x00); // returned byte should match default of register map unless edited manually (Datasheet, pg.39)
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH); //High to end communication
//     SPI.endTransaction();
//     return data;
// }

// /**
//  * Read _numRegisters register starting at address _address.
//  * @param _address      start address
//  * @param _numRegisters number of registers
//  * @param data          pointer to data array
//  */
// void ADS129X::RREG(byte _address, byte _numRegisters, byte *_data) {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     byte opcode1 = ADS129X_CMD_RREG | (_address & 0x1F); //001rrrrr; _RREG = 00100000 and _address = rrrrr
//     digitalWrite(CS, LOW); //Low to communicated
//     SPI.transfer(ADS129X_CMD_SDATAC); //SDATAC
//     SPI.transfer(opcode1); //RREG
//     SPI.transfer(_numRegisters-1); //opcode2
//     for(byte i = 0; i < _numRegisters; i++){
//         *(_data+i) = SPI.transfer(0x00); // returned byte should match default of register map unless previously edited manually (Datasheet, pg.39)
//     }
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH); //High to end communication
//     SPI.endTransaction();
// }

// /**
//  * Write register at address _address.
//  * @param _address register address
//  * @param _value   register value
//  */
// void ADS129X::WREG(byte _address, byte _value) {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     byte opcode1 = ADS129X_CMD_WREG | (_address & 0x1F); //001rrrrr; _RREG = 00100000 and _address = rrrrr
//     digitalWrite(CS, LOW); //Low to communicate
//     SPI.transfer(opcode1);
//     SPI.transfer(0x00); // opcode2; only write one register
//     SPI.transfer(_value);
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH); //Low to communicate
//     SPI.endTransaction();
// }

// /**
//  * Read device ID.
//  * @return device ID
//  */
// byte ADS129X::getDeviceId() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(CS, LOW); //Low to communicate
//     SPI.transfer(ADS129X_CMD_RREG); //RREG
//     SPI.transfer(0x00); //Asking for 1 byte
//     byte data = SPI.transfer(0x00); // byte to read (hopefully 0b???11110)
//     delayMicroseconds(2);
//     digitalWrite(CS, HIGH); //Low to communicate
//     SPI.endTransaction();
//     return data;
// }

// #ifndef ADS129X_POLLING
// /**
//  * Interrupt that gets called when DRDY goes HIGH.
//  * Transfers data and sets a flag.
//  */
// void ADS129X_dataReadyISR() {
//     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(ADS129X_CS, LOW);
//     // status
//     ((char*) ADS129X_data)[0*4+3] = 0;
//     ((char*) ADS129X_data)[0*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[0*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[0*4+0] = SPI.transfer(0x00);
//     // channel 1
//     ((char*) ADS129X_data)[1*4+3] = 0;
//     ((char*) ADS129X_data)[1*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[1*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[1*4+0] = SPI.transfer(0x00);
//     // channel 2
//     ((char*) ADS129X_data)[2*4+3] = 0;
//     ((char*) ADS129X_data)[2*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[2*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[2*4+0] = SPI.transfer(0x00);
//     // channel 3
//     ((char*) ADS129X_data)[3*4+3] = 0;
//     ((char*) ADS129X_data)[3*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[3*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[3*4+0] = SPI.transfer(0x00);
//     // channel 4
//     ((char*) ADS129X_data)[4*4+3] = 0;
//     ((char*) ADS129X_data)[4*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[4*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[4*4+0] = SPI.transfer(0x00);
//     // channel 5
//     ((char*) ADS129X_data)[5*4+3] = 0;
//     ((char*) ADS129X_data)[5*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[5*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[5*4+0] = SPI.transfer(0x00);
//     // channel 6
//     ((char*) ADS129X_data)[6*4+3] = 0;
//     ((char*) ADS129X_data)[6*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[6*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[6*4+0] = SPI.transfer(0x00);
//     // channel 7
//     ((char*) ADS129X_data)[7*4+3] = 0;
//     ((char*) ADS129X_data)[7*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[7*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[7*4+0] = SPI.transfer(0x00);
//     // channel 8
//     ((char*) ADS129X_data)[8*4+3] = 0;
//     ((char*) ADS129X_data)[8*4+2] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[8*4+1] = SPI.transfer(0x00);
//     ((char*) ADS129X_data)[8*4+0] = SPI.transfer(0x00);
//     digitalWrite(ADS129X_CS, HIGH);
//     SPI.endTransaction();
//     ADS129X_newData = true;
// }
// #endif

// /**
//  * Receive data when in continuous read mode.
//  * @param buffer buffer for received data
//  * @return true when received data
//  */
// boolean ADS129X::getData(long *buffer) {
// #ifndef ADS129X_POLLING
//     if (ADS129X_newData) {
//         ADS129X_newData = false;
//         for (int i = 0; i < 9; i++) {
//             buffer[i] = ADS129X_data[i];
//         }
//         return true;
//     }
//     return false;
// #else
//     if (digitalRead(DRDY) == LOW) {
//         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
//         digitalWrite(CS, LOW);
//         for(int i = 0; i<9; i++){
//             long dataPacket = 0;
//             for(int j = 0; j<3; j++){
//                 byte dataByte = SPI.transfer(0x00);
//                 dataPacket = (dataPacket<<8) | dataByte;
//             }
//             buffer[i] = dataPacket;
//         }
//         digitalWrite(CS, HIGH);
//         SPI.endTransaction();
//         return true;
//     }
//     return false;
// #endif
// }

// /**
//  * Configure channel _channel.
//  * @param _channel   channel (1-8)
//  * @param _powerDown power down (true, false)
//  * @param _gain      gain setting
//  * @param _mux       mux setting
//  */
// void ADS129X::configChannel(byte _channel, boolean _powerDown, byte _gain, byte _mux) {
//     byte value = ((_powerDown & 1)<<7) | ((_gain & 7)<<4) | (_mux & 7);
//     WREG(ADS129X_REG_CH1SET + (_channel-1), value);
// }
