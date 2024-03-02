/**
 * @file basic.ino
 * @author Martin Vichnal
 * @brief CAP1188 Capacitive Touch Sensor Firmware using I2C communication method.
 * @version v1.0.2
 * @date 2024-03-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <Wire.h>
#include <SPI.h>
#include <CAP1188.h>

CAP1188 cap = CAP1188();

void setup()
{
    Serial.begin(115200);
    Serial.println("CAP1188 test!");

    // Initialize the sensor, if using i2c you can pass in the i2c address
    // if (!cap.begin(0x28)) {
    if (!cap.begin())
    {
        Serial.println("CAP1188 not found");
        while (1)
            ;
    }
    Serial.println("CAP1188 found!");

    uint8_t sens = cap.readRegister(CAP1188_REG_SENSITIVITY_CONTROL);
    Serial.println("sens: ");
    Serial.println(sens);

    sens = cap.setSensitivity(CAP1188_SENS_VAL_64x);
    Serial.println("sens: ");
    Serial.println(sens);
}

void loop()
{
    uint8_t touched = cap.touched();

    if (touched == 0)
    {
        // No touch detected
        return;
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        if (touched & (1 << i))
        {
            Serial.print("C");
            Serial.print(i + 1);
            Serial.print("\t");
        }
    }
    // Serial.println(touched, BIN);
    Serial.println();
    delay(50);
}