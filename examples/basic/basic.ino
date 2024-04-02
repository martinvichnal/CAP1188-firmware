/**
 * @file basic.ino
 * @author Martin Vichnal
 * @brief CAP1188 Capacitive Touch Sensor Firmware using I2C communication method.
 * @version v1.1.0
 * @date 2024-04-02
 *
 * @copyright Copyright (c) 2024
 *
 * INFO:
 *  - Avarage low pulse interrupt time: 1.29 ms
 */

#include <Wire.h>
#include <SPI.h>
#include <CAP1188.h>
#include <AntiDelay.h>

#define BUTTON_Mode 1
#define BUTTON_Minus 0
#define BUTTON_Plus 2

#define CAP1199_RESET_PIN 13

void capInit();
void capInterruptHandler();

AntiDelay anitDelay = AntiDelay(1);
CAP1188 cap = CAP1188(CAP1199_RESET_PIN);

uint8_t globalSensorState = 0;
volatile bool interruptFlag = false;

struct Button
{
	uint8_t pin;	// The pin number
	char name[100]; // The name of the button
	bool state;		// The state of the button
};

Button buttons[] = {
	{0, "Minus", false},
	{1, "Mode", false},
	{2, "Plus", false}};

void setup()
{
	Wire.begin();

	Serial.begin(115200);
	Serial.println("CAP1188 test!");
	pinMode(14, INPUT);
	attachInterrupt(digitalPinToInterrupt(14), capInterruptHandler, FALLING);

	if (!cap.begin())
	{
		while (!cap.resetCAP1188())
		{
			Serial.println("CAP1188 is not detected!");
			delay(1000);
		}
		cap.begin();
	}
	else
	{
		Serial.println("CAP1188 is detected!");
		capInit();
	}
}

void loop()
{
	if (interruptFlag)
	{
		bool currentInterruptFlag = interruptFlag;
		interruptFlag = false;

		if (currentInterruptFlag)
		{
			globalSensorState = cap.touched();
			uint8_t sensorState = globalSensorState;
			for (uint8_t i = 0; i < 3; i++)
			{
				bool newState = sensorState & (1 << buttons[i].pin);
				if (newState != buttons[i].state)
				{
					buttons[i].state = newState;
				}
			}
		}
	}

	if (anitDelay)
	{
		Serial.print("The global sensor state (in hex) is: \t");
		Serial.println(globalSensorState, HEX);
	}
}

void capInit()
{
	// Setting the sensitivity of the sensor
	uint8_t sens = cap.setSensitivity(CAP1188_SENS_VAL_128x);
	Serial.println("sens: ");
	Serial.println(sens, HEX);
	// Disable unwanted inputs
	cap.writeRegister(CAP1188_REG_SENSOR_INPUT_ENABLE, 0x07);
	// Enable first 3 touch pad interrupts
	cap.writeRegister(CAP1188_REG_INTERRUPT_ENABLE, 0x07);
	// Disable repeate rate for interrupt generation (interrupt only generated when the button state changes)
	cap.writeRegister(CAP1188_REG_REPEAT_RATE_ENABLE, 0x00);
	// Set stand by channel for 3 touch pads
	cap.writeRegister(CAP1188_REG_STANDBY_CHANNEL, 0x07);
	// Flipping the interrupt pin just in case
	cap.writeRegister(CAP1188_REG_MAIN_CONTROL, cap.readRegister(CAP1188_REG_MAIN_CONTROL) & ~CAP1188_REG_MAIN_INT);
}

void capInterruptHandler()
{
	interruptFlag = true;
}