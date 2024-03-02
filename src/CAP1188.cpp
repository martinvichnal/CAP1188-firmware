/*!
 *  @file Adafruit_CAP1188.cpp
 *
 *  @mainpage Adafruit CAP1188 I2C/SPI 8-chan Capacitive Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	This is a library for the Adafruit CAP1188 I2C/SPI 8-chan Capacitive
 * Sensor http://www.adafruit.com/products/1602
 *
 *  These sensors use I2C/SPI to communicate, 2+ pins are required to
 *  interface
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "CAP1188.h"

/*!
 *    @brief  Instantiates a new CAP1188 class using hardware I2C
 *    @param  resetpin
 *            number of pin where reset is connected
 *
 */
CAP1188::CAP1188(int8_t resetpin)
{
    _resetpin = resetpin;
}


boolean CAP1188::begin(uint8_t i2caddr, TwoWire *theWire)
{
    i2c_dev = new Adafruit_I2CDevice(i2caddr, theWire);
    if (!i2c_dev->begin())
    {
        return false;
    }

    if (_resetpin != -1)
    {
        pinMode(_resetpin, OUTPUT);
        digitalWrite(_resetpin, LOW);
        delay(100);
        digitalWrite(_resetpin, HIGH);
        delay(100);
        digitalWrite(_resetpin, LOW);
        delay(100);
    }

    readRegister(CAP1188_REG_PRODUCT_ID);

    // Useful debugging info

    Serial.print("Product ID: 0x");
    Serial.println(readRegister(CAP1188_REG_PRODUCT_ID), HEX);
    Serial.print("Manuf. ID: 0x");
    Serial.println(readRegister(CAP1188_REG_MANUFACTURER_ID), HEX);
    Serial.print("Revision: 0x");
    Serial.println(readRegister(CAP1188_REG_REVISION), HEX);

    if ((readRegister(CAP1188_REG_PRODUCT_ID) != 0x50) ||
        (readRegister(CAP1188_REG_MANUFACTURER_ID) != 0x5D) ||
        (readRegister(CAP1188_REG_REVISION) != 0x83))
    {
        return false;
    }
    // allow multiple touches
    writeRegister(CAP1188_REG_MULTIPLE_TOUCH_CONFIG, 0);
    // Have LEDs follow touches
    writeRegister(CAP1188_REG_SENSOR_INPUT_LED_LINKING, 0x00);
    // speed up a bit
    writeRegister(CAP1188_REG_STANDBY_CONFIGURATION, 0x30);
    return true;
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
    uint8_t sensMask = 0x8F;    // 10001111
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
uint8_t CAP1188::touched()
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
    uint8_t buffer[3] = {reg, 0, 0};
    i2c_dev->write_then_read(buffer, 1, buffer, 1);
    return buffer[0];
}

/**
 * @brief Sets specific register bits with mask
 * 
 * @param reg - Register address
 * @param mask - Mask value to clear specific bits 0: clear, 1: leave
 * @param value - Specific bit to set in the register
 * @return u_int8_t - New register bits
 */
u_int8_t CAP1188::setRegister(uint8_t reg, uint8_t mask, uint8_t value)
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
    uint8_t buffer[4] = {reg, value, 0, 0};
    i2c_dev->write(buffer, 2);
}