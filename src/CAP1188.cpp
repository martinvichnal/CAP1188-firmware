/**
 * @file CAP1188.cpp
 * @author Martin Vichnal
 * @brief CAP1188 Capacitive Touch Sensor Firmware using I2C communication method.
 * @version v1.1.1
 * @date 2024-05-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "CAP1188.h"

/**
 * @brief Construct a new CAP1188::CAP1188 object
 *
 * @param resetpin
 */
CAP1188::CAP1188(int8_t resetpin)
{
    _resetpin = resetpin;
}

/**
 * @brief Begins the CAP1188 touch sensor, restarts and checks the integrity of the device. Sets basic configuration.
 *
 * @return bool - True if the device is detected and the integrity is checked, otherwise false.
 */
bool CAP1188::begin()
{
    bool res = false;

    res = reset();

    return res;
}

/**
 * @brief Sets the sensitivity value for the CAP1188 touch sensor.
 * @note Best is either 64 or 128
 *
 * @param value - Sensitivity value - CAP1188_SENS_VAL_(DEF_32x/64x/128x)
 * @return uint8_t - Updated sensitivity value read from the sensitivity control register.
 */
uint8_t CAP1188::setSensitivity(uint8_t value)
{
    uint8_t sensMask = 0x8F; // 10001111
    uint8_t res = readRegister(CAP1188_REG_SENSITIVITY_CONTROL);
    res &= sensMask;
    res |= value;
    writeRegister(CAP1188_REG_SENSITIVITY_CONTROL, res);

    return (res = readRegister(CAP1188_REG_SENSITIVITY_CONTROL));
}

/**
 * @brief Reads the sensor status register containing touched inputs
 *
 * @return uint8_t - Returns touched bits 1 - touched, 0 - not touched.
 */
uint8_t CAP1188::getTouch()
{
    uint8_t t = readRegister(CAP1188_REG_SENSOR_INPUT_STATUS);
    if (t)
    {
        writeRegister(CAP1188_REG_MAIN_CONTROL, readRegister(CAP1188_REG_MAIN_CONTROL) & ~CAP1188_REG_MAIN_INT);
    }
    return t;
}

/**
 * @brief Reads the value from the specified register
 *
 * @param reg - Register address.
 * @return uint8_t - The value read from the register.
 */
uint8_t CAP1188::readRegister(uint8_t reg)
{
    uint8_t res = 0;

    Wire.beginTransmission(CAP1188_DEFAULT_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(CAP1188_DEFAULT_ADDRESS, 1);
    res = Wire.read();
    return res;
}

/**
 * @brief Sets specific register bits with mask
 *
 * @param reg - Register address
 * @param mask - Mask value to clear specific bits 0: clear, 1: leave
 * @param value - Specific bit to set in the register
 * @return u_int8_t - New register bits
 */
uint8_t CAP1188::setRegister(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t res = readRegister(reg);
    res &= mask;
    res |= value;
    writeRegister(reg, res);

    return (res = readRegister(reg));
}

/**
 * @brief Writes 8-bits to the specified destination register
 *
 * @param reg - Register address
 * @param value - Value that will be written at selected register
 */
void CAP1188::writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(CAP1188_DEFAULT_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

/**
 * @brief Resets the interrupt bit
 *
 *
 * @param 0 on default (value The interrupt value to be set.)
 */
void CAP1188::setInt(uint8_t value)
{
    writeRegister(CAP1188_REG_MAIN_CONTROL, value);
}

/**
 * @brief Check integrity of the device with manifacturer and product predefined values
 *
 * @return bool - True if the device is detected and the integrity is checked, otherwise false.
 */
bool CAP1188::checkIntegrity()
{
    bool res = false;

    if ((readRegister(CAP1188_REG_PRODUCT_ID) == 0x50) && (readRegister(CAP1188_REG_MANUFACTURER_ID) == 0x5D) && (readRegister(CAP1188_REG_REVISION) == 0x83))
        res = true;
    else
        res = false;

    return res;
}

/**
 * @brief Resets the CAP1188 device
 *
 * @return bool - True if the device is detected and the integrity is checked, otherwise false.
 */
bool CAP1188::reset()
{
    bool res = false;

    if (_resetpin != -1)
    {
        pinMode(_resetpin, OUTPUT);
        digitalWrite(_resetpin, LOW);
        delay(100);
        digitalWrite(_resetpin, HIGH);
        delay(100);
        digitalWrite(_resetpin, LOW);
        delay(100);
        setInt(); // Clear the interrupt bit
    }

    res = checkIntegrity();

    writeRegister(CAP1188_REG_MULTIPLE_TOUCH_CONFIG, 0x00);    // allow multiple touches
    writeRegister(CAP1188_REG_SENSOR_INPUT_LED_LINKING, 0x00); // Have LEDs follow touches
    writeRegister(CAP1188_REG_STANDBY_CONFIGURATION, 0x30);    // speed up a bit
    setSensitivity(CAP1188_SENS_VAL_128x);                     // Setting the sensitivity of the sensor
    writeRegister(CAP1188_REG_SENSOR_INPUT_ENABLE, 0x07);      // Disable unwanted inputs
    writeRegister(CAP1188_REG_INTERRUPT_ENABLE, 0x07);         // Enable first 3 touch pad interrupts
    writeRegister(CAP1188_REG_REPEAT_RATE_ENABLE, 0x00);       // Disable repeate rate for interrupt generation (interrupt only generated when the button state changes)
    writeRegister(CAP1188_REG_STANDBY_CHANNEL, 0x07);          // Set stand by channel for 3 touch pads

    setInt(); // Flipping the interrupt pin just in case

    return res;
}