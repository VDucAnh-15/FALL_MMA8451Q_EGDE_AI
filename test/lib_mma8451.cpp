/**
 * MMA8451 Accelerometer Library - Version 2
 * Rewritten based on NXP AN4073 Application Note
 *
 * Critical fixes:
 * - Proper FIFO initialization sequence
 * - Correct register read/write timing
 * - Proper Active/Standby mode transitions
 */

#include "lib_mma8451.h"
#include <Wire.h>

MMA8451::MMA8451(uint8_t addr)
{
    i2c_address = addr;
    current_range = 0; // ±2g default
    is_active = false;
}

bool MMA8451::begin()
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C

    delay(50); // Wait for sensor power-up

    // Check WHO_AM_I
    uint8_t id = readRegister(REG_WHO_AM_I);
    // Serial.print("WHO_AM_I = 0x");
    // Serial.println(id, HEX);

    if (id == MMA8451_ID || id == MMA8451_ID_ALT)
    {
        // Serial.println("MMA8451 detected!");

        // Put into Standby for configuration
        enterStandby();

        return true;
    }

    // Serial.println("ERROR: MMA8451 not found!");
    return false;
}

void MMA8451::enterStandby()
{
    uint8_t ctrl1 = readRegister(REG_CTRL_REG1);
    ctrl1 &= ~0x01; // Clear Active bit
    writeRegister(REG_CTRL_REG1, ctrl1);
    delay(2); // Wait for standby
    is_active = false;

    // Serial.println("Entered Standby mode");
}

void MMA8451::enterActive()
{
    uint8_t ctrl1 = readRegister(REG_CTRL_REG1);
    ctrl1 |= 0x01; // Set Active bit
    writeRegister(REG_CTRL_REG1, ctrl1);
    delay(2); // Wait for active
    is_active = true;

    // Serial.println("Entered Active mode");
}

/**
 * Setup FIFO with Watermark Interrupt
 * Based on AN4073 Section 3: FIFO Operating Modes
 *
 * Sequence (MUST be in Standby mode):
 * 1. Set data rate in CTRL_REG1
 * 2. Set range in XYZ_DATA_CFG
 * 3. Configure F_SETUP (mode + watermark)
 * 4. Enable interrupt in CTRL_REG4
 * 5. Route interrupt in CTRL_REG5
 * 6. Go to Active mode
 */
void MMA8451::setupFIFO_Watermark(uint8_t data_rate, uint8_t range, uint8_t watermark)
{
    // Serial.println("\n=== FIFO Watermark Setup (AN4073) ===");

    // Validate watermark (1-32)
    if (watermark == 0)
        watermark = 1;
    if (watermark > 32)
        watermark = 32;

    // Serial.print("Config: DR=");
    // Serial.print(data_rate);
    // Serial.print(", Range=");
    // Serial.print(range);
    // Serial.print(", Watermark=");
    // Serial.println(watermark);

    // MUST be in Standby
    enterStandby();

    // Step 1: Set Data Rate (DR bits [5:3])
    // Active=0, F_READ=0 (14-bit mode)
    uint8_t ctrl1 = (data_rate << 3) & 0x38;
    writeRegister(REG_CTRL_REG1, ctrl1);
    delay(2);
    // Serial.print("CTRL_REG1 = 0x");
    // Serial.println(readRegister(REG_CTRL_REG1), HEX);

    // Step 2: Set Range
    current_range = range;
    writeRegister(REG_XYZ_DATA_CFG, range);
    delay(2);
    // Serial.print("XYZ_DATA_CFG = 0x");
    // Serial.println(readRegister(REG_XYZ_DATA_CFG), HEX);

    // Step 3: Configure FIFO
    // F_MODE[7:6] = 01 (Circular mode)
    // WMRK[5:0] = watermark value (interrupt triggers when F_CNT > WMRK)
    // Để có 25 samples, cần ghi WMRK = 24 (25 - 1)
    // QUAN TRỌNG: Phải ghi chính xác (watermark - 1), KHÔNG shift thêm
    uint8_t f_setup = (0x01 << 6) | (watermark & 0x3F);
    writeRegister(REG_F_SETUP, f_setup);
    delay(2);

    uint8_t f_setup_check = readRegister(REG_F_SETUP);
    // Serial.print("F_SETUP write=0x");
    // Serial.print(f_setup, HEX);
    // Serial.print(" (mode=");
    // Serial.print((f_setup >> 6) & 0x03);
    // Serial.print(", wmrk=");
    // Serial.print((f_setup & 0x3F) + 1);
    // Serial.print(") read=0x");
    // Serial.println(f_setup_check, HEX);

    if (f_setup_check != f_setup)
    {
        // Serial.println("WARNING: F_SETUP readback mismatch!");
        // Continue anyway - don't return
    }

    // Step 4: Enable FIFO Interrupt (bit 6 of CTRL_REG4)
    writeRegister(REG_CTRL_REG4, 0x40);
    delay(2);
    // Serial.print("CTRL_REG4 = 0x");
    // Serial.println(readRegister(REG_CTRL_REG4), HEX);

    // Step 5: Route to INT1 (bit 6 of CTRL_REG5)
    writeRegister(REG_CTRL_REG5, 0x40);
    delay(2);
    // Serial.print("CTRL_REG5 = 0x");
    // Serial.println(readRegister(REG_CTRL_REG5), HEX);

    // Configure interrupt pin mode (push-pull, active low)
    configureInterruptPinMode();

    // Step 6: Go to Active mode
    enterActive();

    // Wait for first samples to accumulate
    // At 50Hz, 1 sample = 20ms. Wait for 3 samples = 60ms
    // delay(100);

    // Test F_STATUS
    // uint8_t f_status = readRegister(REG_F_STATUS);
    // Serial.print("F_STATUS after Active = 0x");
    // Serial.print(f_status, HEX);
    // Serial.print(" (count=");
    // Serial.print(f_status & 0x3F);
    // Serial.println(")");

    // Serial.println("=== Setup Complete ===\n");
}

uint8_t MMA8451::getFIFOCount()
{
    // Read F_STATUS directly without checking is_active flag
    // The flag may not be reliable after setup
    uint8_t f_status = readRegister(REG_F_STATUS);

    // Check for errors
    if (f_status == 0xFF)
    {
        // Serial.println("ERROR: F_STATUS = 0xFF (I2C error or FIFO not enabled)");
        return 0;
    }

    uint8_t count = f_status & 0x3F; // Bits [5:0]

    if (count > 32)
    {
        // Serial.print("ERROR: Invalid FIFO count = ");
        // Serial.println(count);
        return 0;
    }

    return count;
}

void MMA8451::configureInterruptPinMode()
{
    // Configure CTRL_REG3 for interrupt pin behavior
    // Bit 1 (IPOL): 0 = Active Low (for ESP32 FALLING trigger)
    // Bit 0 (PP_OD): 0 = Push-Pull output
    writeRegister(REG_CTRL_REG3, 0x00);
    delay(2);
}

void MMA8451::clearFIFOInterrupt()
{
    // Clear interrupt by reading INT_SOURCE register
    readRegister(REG_INT_SOURCE);
}

void MMA8451::readFIFOSample(float &x, float &y, float &z)
{
    // Read 6 bytes: X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB
    Wire.beginTransmission(i2c_address);
    Wire.write(REG_OUT_X_MSB);
    Wire.endTransmission(false);

    Wire.requestFrom(i2c_address, (uint8_t)6);

    if (Wire.available() >= 6)
    {
        uint8_t xhi = Wire.read();
        uint8_t xlo = Wire.read();
        uint8_t yhi = Wire.read();
        uint8_t ylo = Wire.read();
        uint8_t zhi = Wire.read();
        uint8_t zlo = Wire.read();

        // 14-bit data (left-justified in 16-bit)
        int16_t x_raw = (int16_t)((xhi << 8) | xlo) >> 2;
        int16_t y_raw = (int16_t)((yhi << 8) | ylo) >> 2;
        int16_t z_raw = (int16_t)((zhi << 8) | zlo) >> 2;

        // Convert to g based on range
        float scale;
        switch (current_range)
        {
        case 0:
            scale = 2.0 / 8192.0;
            break; // ±2g
        case 1:
            scale = 4.0 / 8192.0;
            break; // ±4g
        case 2:
            scale = 8.0 / 8192.0;
            break; // ±8g
        default:
            scale = 2.0 / 8192.0;
            break;
        }

        x = x_raw * scale;
        y = y_raw * scale;
        z = z_raw * scale;
    }
    else
    {
        x = y = z = 0;
        // Serial.println("ERROR: Failed to read sample");
    }
}

void MMA8451::readFIFOSampleRaw(int16_t &x, int16_t &y, int16_t &z)
{
    // Read 6 bytes: X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB
    Wire.beginTransmission(i2c_address);
    Wire.write(REG_OUT_X_MSB);
    Wire.endTransmission(false);

    Wire.requestFrom(i2c_address, (uint8_t)6);

    if (Wire.available() >= 6)
    {
        uint8_t xhi = Wire.read();
        uint8_t xlo = Wire.read();
        uint8_t yhi = Wire.read();
        uint8_t ylo = Wire.read();
        uint8_t zhi = Wire.read();
        uint8_t zlo = Wire.read();

        // 14-bit data (left-justified in 16-bit)
        x = (int16_t)((xhi << 8) | xlo) >> 2;
        y = (int16_t)((yhi << 8) | ylo) >> 2;
        z = (int16_t)((zhi << 8) | zlo) >> 2;
    }
    else
    {
        x = y = z = 0;
        // Serial.println("ERROR: Failed to read sample");
    }
}

void MMA8451::printStatus()
{
    Serial.println("\n========== MMA8451 Status ==========");

    uint8_t sysmod = readRegister(REG_SYSMOD);
    Serial.print("SYSMOD (0x0B): 0x");
    Serial.print(sysmod, HEX);
    Serial.print(" - ");
    switch (sysmod & 0x03)
    {
    case 0:
        Serial.println("STANDBY");
        break;
    case 1:
        Serial.println("WAKE");
        break;
    case 2:
        Serial.println("SLEEP");
        break;
    default:
        Serial.println("UNKNOWN");
        break;
    }

    uint8_t f_status = readRegister(REG_F_STATUS);
    Serial.print("F_STATUS (0x00): 0x");
    Serial.print(f_status, HEX);
    Serial.print(" = 0b");
    for (int i = 7; i >= 0; i--)
    {
        Serial.print((f_status >> i) & 1);
    }
    Serial.println();
    Serial.print("  F_OVF (overflow): ");
    Serial.println((f_status & 0x80) ? "YES" : "NO");
    Serial.print("  F_WMRK (watermark): ");
    Serial.println((f_status & 0x40) ? "YES" : "NO");
    Serial.print("  F_CNT (samples): ");
    Serial.println(f_status & 0x3F);

    uint8_t f_setup = readRegister(REG_F_SETUP);
    Serial.print("F_SETUP (0x09): 0x");
    Serial.print(f_setup, HEX);
    Serial.print(" - Mode=");
    uint8_t mode = (f_setup >> 6) & 0x03;
    switch (mode)
    {
    case 0:
        Serial.print("Disabled");
        break;
    case 1:
        Serial.print("Circular");
        break;
    case 2:
        Serial.print("Fill");
        break;
    case 3:
        Serial.print("Trigger");
        break;
    }
    Serial.print(", Watermark=");
    Serial.println((f_setup & 0x3F) + 1);

    uint8_t ctrl1 = readRegister(REG_CTRL_REG1);
    Serial.print("CTRL_REG1 (0x2A): 0x");
    Serial.print(ctrl1, HEX);
    Serial.print(" - Active=");
    Serial.print((ctrl1 & 0x01) ? "YES" : "NO");
    Serial.print(", DR=");
    Serial.println((ctrl1 >> 3) & 0x07);

    uint8_t ctrl4 = readRegister(REG_CTRL_REG4);
    Serial.print("CTRL_REG4 (0x2D): 0x");
    Serial.print(ctrl4, HEX);
    Serial.print(" - INT_EN_FIFO=");
    Serial.println((ctrl4 & 0x40) ? "YES" : "NO");

    uint8_t ctrl5 = readRegister(REG_CTRL_REG5);
    Serial.print("CTRL_REG5 (0x2E): 0x");
    Serial.print(ctrl5, HEX);
    Serial.print(" - INT_CFG_FIFO=");
    Serial.println((ctrl5 & 0x40) ? "INT1" : "INT2");

    Serial.println("====================================\n");
}

// Low-level I2C functions
uint8_t MMA8451::readRegister(uint8_t reg)
{
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    uint8_t error = Wire.endTransmission(false);

    if (error != 0)
    {
        // Serial.print("I2C TX error: ");
        // Serial.println(error);
        return 0xFF;
    }

    Wire.requestFrom(i2c_address, (uint8_t)1);

    if (Wire.available())
    {
        return Wire.read();
    }

    // Serial.println("I2C RX error");
    return 0xFF;
}

void MMA8451::writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.write(value);
    uint8_t error = Wire.endTransmission();

    if (error != 0)
    {
        // Serial.print("I2C write error: ");
        // Serial.println(error);
    }
}
