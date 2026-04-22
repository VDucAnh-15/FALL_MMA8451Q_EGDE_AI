/**
 * MMA8451 Accelerometer Library - Version 2
 * Rewritten based on NXP AN4073 Application Note
 */

#ifndef LIB_MMA8451_H
#define LIB_MMA8451_H

#include <Arduino.h>

// Data Rate Options (DR[2:0] in CTRL_REG1[5:3])
#define DR_800HZ 0  // 800 Hz
#define DR_400HZ 1  // 400 Hz
#define DR_200HZ 2  // 200 Hz
#define DR_100HZ 3  // 100 Hz
#define DR_50HZ 4   // 50 Hz
#define DR_12_5HZ 5 // 12.5 Hz
#define DR_6_25HZ 6 // 6.25 Hz
#define DR_1_56HZ 7 // 1.56 Hz

// Range Options (FS[1:0] in XYZ_DATA_CFG[1:0])
#define RANGE_2G 0 // ±2g
#define RANGE_4G 1 // ±4g
#define RANGE_8G 2 // ±8g

// Register Addresses (from MMA8451Q datasheet)
#define REG_STATUS 0x00   // Read-only
#define REG_F_STATUS 0x00 // Same as STATUS when FIFO enabled
#define REG_OUT_X_MSB 0x01
#define REG_OUT_X_LSB 0x02
#define REG_OUT_Y_MSB 0x03
#define REG_OUT_Y_LSB 0x04
#define REG_OUT_Z_MSB 0x05
#define REG_OUT_Z_LSB 0x06
#define REG_F_SETUP 0x09
#define REG_TRIG_CFG 0x0A
#define REG_SYSMOD 0x0B
#define REG_INT_SOURCE 0x0C
#define REG_WHO_AM_I 0x0D
#define REG_XYZ_DATA_CFG 0x0E
#define REG_HP_FILTER_CUTOFF 0x0F
#define REG_CTRL_REG1 0x2A
#define REG_CTRL_REG2 0x2B
#define REG_CTRL_REG3 0x2C
#define REG_CTRL_REG4 0x2D
#define REG_CTRL_REG5 0x2E

// WHO_AM_I value
#define MMA8451_ID 0x1A     // Standard ID
#define MMA8451_ID_ALT 0x2A // Alternative ID (some variants)

class MMA8451
{
public:
    MMA8451(uint8_t addr = 0x1C);

    // Initialization
    bool begin();

    // Mode control
    void enterStandby();
    void enterActive();

    // FIFO Configuration (AN4073 compliant)
    void setupFIFO_Watermark(uint8_t data_rate, uint8_t range, uint8_t watermark);

    // Interrupt Configuration for low power
    void configureInterruptPinMode();

    // FIFO Operations
    uint8_t getFIFOCount();
    void readFIFOSample(float &x, float &y, float &z);
    void readFIFOSampleRaw(int16_t &x, int16_t &y, int16_t &z); // Raw data version
    void clearFIFOInterrupt();

    // Debug
    void printStatus();

    // Low-level access
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);

private:
    uint8_t i2c_address;
    uint8_t current_range;
    bool is_active;
};

#endif
