/**
 * @file CAP1188.h
 * @author Martin Vichnal
 * @brief CAP1188 Capacitive Touch Sensor Firmware using I2C communication method.
 * @version v1.1.0
 * @date 2024-04-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Arduino.h"
#include <Wire.h>

#ifndef _CAP1188_H
#define _CAP1188_H

#define CAP1188_DEFAULT_ADDRESS 0x28

#define CAP1188_REG_MAIN_CONTROL 0x00
#define CAP1188_REG_MAIN_INT 0x01
#define CAP1188_REG_GENERAL_STATUS 0x02
#define CAP1188_REG_SENSOR_INPUT_STATUS 0x03
#define CAP1188_REG_LED_STATUS 0x04
#define CAP1188_REG_NOISE_FLAG_STATUS 0x0A
#define CAP1188_REG_SENSOR_INPUT_1_DELTA 0x10
#define CAP1188_REG_SENSOR_INPUT_2_DELTA 0x11
#define CAP1188_REG_SENSOR_INPUT_3_DELTA 0x12
#define CAP1188_REG_SENSOR_INPUT_4_DELTA 0x13
#define CAP1188_REG_SENSOR_INPUT_5_DELTA 0x14
#define CAP1188_REG_SENSOR_INPUT_6_DELTA 0x15
#define CAP1188_REG_SENSOR_INPUT_7_DELTA 0x16
#define CAP1188_REG_SENSOR_INPUT_8_DELTA 0x17
#define CAP1188_REG_SENSITIVITY_CONTROL 0x1F
#define CAP1188_REG_CONFIGURATION 0x20
#define CAP1188_REG_SENSOR_INPUT_ENABLE 0x21
#define CAP1188_REG_SENSOR_INPUT_CONFIG 0x22
#define CAP1188_REG_SENSOR_INPUT_CONFIG2 0x23
#define CAP1188_REG_AVG_AND_SAMPLE_CONFIG 0x24
#define CAP1188_REG_CALIBRATION_ACTIVATE 0x26
#define CAP1188_REG_INTERRUPT_ENABLE 0x27
#define CAP1188_REG_REPEAT_RATE_ENABLE 0x28
#define CAP1188_REG_MULTIPLE_TOUCH_CONFIG 0x2A
#define CAP1188_REG_MULTI_TOUCH_PATTERN_CONFIG 0x2B
#define CAP1188_REG_MULTI_TOUCH_PATTERN 0x2D
#define CAP1188_REG_RECALIBRATION_CONFIG 0x2F
#define CAP1188_REG_SENSOR_INPUT_1_THRESHOLD 0x30
#define CAP1188_REG_SENSOR_INPUT_2_THRESHOLD 0x31
#define CAP1188_REG_SENSOR_INPUT_3_THRESHOLD 0x32
#define CAP1188_REG_SENSOR_INPUT_4_THRESHOLD 0x33
#define CAP1188_REG_SENSOR_INPUT_5_THRESHOLD 0x34
#define CAP1188_REG_SENSOR_INPUT_6_THRESHOLD 0x35
#define CAP1188_REG_SENSOR_INPUT_7_THRESHOLD 0x36
#define CAP1188_REG_SENSOR_INPUT_8_THRESHOLD 0x37
#define CAP1188_REG_SENSOR_INPUT_NOISE_THRESHOLD 0x38
// Standby Configuration Registers
#define CAP1188_REG_STANDBY_CHANNEL 0x40
#define CAP1188_REG_STANDBY_CONFIGURATION 0x41
#define CAP1188_REG_STANDBY_SENSITIVITY 0x42
#define CAP1188_REG_STANDBY_THRESHOLD 0x43

#define CAP1188_REG_CONFIGURATION_2 0x44
// BASECOUNT REGISTERS
#define CAP1188_REG_SENSOR_INPUT_1_BASECOUNT 0x50
#define CAP1188_REG_SENSOR_INPUT_2_BASECOUNT 0x51
#define CAP1188_REG_SENSOR_INPUT_3_BASECOUNT 0x52
#define CAP1188_REG_SENSOR_INPUT_4_BASECOUNT 0x53
#define CAP1188_REG_SENSOR_INPUT_5_BASECOUNT 0x54
#define CAP1188_REG_SENSOR_INPUT_6_BASECOUNT 0x55
#define CAP1188_REG_SENSOR_INPUT_7_BASECOUNT 0x56
#define CAP1188_REG_SENSOR_INPUT_8_BASECOUNT 0x57
// LED CONTROLS
#define CAP1188_REG_LED_OUTPUT_TYPE 0x71
#define CAP1188_REG_SENSOR_INPUT_LED_LINKING 0x72
#define CAP1188_REG_LED_POLARITY 0x73
#define CAP1188_REG_LED_OUTPUT_CONTROL 0x74
#define CAP1188_REG_LED_LINKED_TRANSITION_CONTROL 0x77
#define CAP1188_REG_LED_MIRROR_CONTROL 0x79
#define CAP1188_REG_LED_BEHAVIOR_1 0x81
#define CAP1188_REG_LED_BEHAVIOR_2 0x82
#define CAP1188_REG_LED_PULSE_1_PERIOD 0x84
#define CAP1188_REG_LED_PULSE_2_PERIOD 0x85
#define CAP1188_REG_LED_BREATHE_PERIOD 0x86
#define CAP1188_REG_LED_CONFIG 0x88
#define CAP1188_REG_LED_PULSE_1_DUTY_CYCLE 0x90
#define CAP1188_REG_LED_PULSE_2_DUTY_CYCLE 0x91
#define CAP1188_REG_LED_BREATHE_DUTY_CYCLE 0x92
#define CAP1188_REG_LED_DIRECT_DUTY_CYCLE 0x93
#define CAP1188_REG_LED_DIRECT_RAMP_RATES 0x94
#define CAP1188_REG_LED_OFF_DELAY 0x95

#define CAP1188_REG_SENSOR_INPUT_1_CALIBRATION 0xB1
#define CAP1188_REG_SENSOR_INPUT_2_CALIBRATION 0xB2
#define CAP1188_REG_SENSOR_INPUT_3_CALIBRATION 0xB3
#define CAP1188_REG_SENSOR_INPUT_4_CALIBRATION 0xB4
#define CAP1188_REG_SENSOR_INPUT_5_CALIBRATION 0xB5
#define CAP1188_REG_SENSOR_INPUT_6_CALIBRATION 0xB6
#define CAP1188_REG_SENSOR_INPUT_7_CALIBRATION 0xB7
#define CAP1188_REG_SENSOR_INPUT_8_CALIBRATION 0xB8

#define CAP1188_REG_SENSOR_INPUT_CALIBRATION_1 0xB9
#define CAP1188_REG_SENSOR_INPUT_CALIBRATION_2 0xBA
#define CAP1188_REG_PRODUCT_ID 0xFD
#define CAP1188_REG_MANUFACTURER_ID 0xFE
#define CAP1188_REG_REVISION 0xFF

#define CAP1188_SENS_VAL_128x 0x00	  // Most sensitive  0bx000.xxxx
#define CAP1188_SENS_VAL_64x 0x10	  // Less sensitive  0bx001.xxxx
#define CAP1188_SENS_VAL_DEF_32x 0x20 // Default         0bx010.xxxx

class CAP1188
{
public:
	CAP1188(int8_t resetpin = -1);

	bool begin();
	bool resetCAP1188();
	bool checkIntegrity();

	uint8_t readRegister(uint8_t reg);
	u_int8_t setRegister(uint8_t reg, uint8_t mask, uint8_t value);
	void writeRegister(uint8_t reg, uint8_t value);

	uint8_t setSensitivity(uint8_t value);
	uint8_t getTouch();

private:
	int8_t _resetpin;
};

#endif
