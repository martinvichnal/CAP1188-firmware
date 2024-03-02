/***************************************************
  This is a library for the CAP1188 I2C/SPI 8-chan Capacitive Sensor

  Designed specifically to work with the CAP1188 sensor from Adafruit
  ----> https://www.adafruit.com/products/1602

  These sensors use I2C/SPI to communicate, 2+ pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

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