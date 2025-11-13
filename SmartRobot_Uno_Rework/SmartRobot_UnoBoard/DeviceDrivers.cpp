#include "DeviceDrivers.h"
#include <avr/wdt.h>
#include <Arduino.h>

namespace sr
{
	uint8_t DeviceDriver_Key::keyValue = 0;

	static void attachPinChangeInterrupt_GetKeyValue(void)
	{
		DeviceDriver_Key ddKey;
		static uint32_t keyValue_time = 0;
		static uint8_t keyValue_tmp = 0;
		if ((millis() - keyValue_time) > 500)
		{
			keyValue_tmp++;
			keyValue_time = millis();
			if (keyValue_tmp > KEYVALUE_MAX) keyValue_tmp = 0;

			ddKey.keyValue = keyValue_tmp;
		}
	}

	void DeviceDriver_Key::KeyInit()
	{
		pinMode(PIN_KEY, INPUT_PULLUP);

		attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING);
	}


	void DeviceDriver_Voltage::VoltageInit()
	{
		pinMode(PIN_VOLTAGE, INPUT);
	}

	float DeviceDriver_Voltage::getAnalogue()
	{
		float volt = (analogRead(PIN_VOLTAGE) * 0.0375);

		volt = volt + (volt * 0.08);

		return volt;
	}

	void DeviceDriver_Motor::MotorInit()
	{
		pinMode(PIN_MOTOR_PWMA, OUTPUT);
		pinMode(PIN_MOTOR_PWMB, OUTPUT);
		pinMode(PIN_MOTOR_AIN_1, OUTPUT);
		pinMode(PIN_MOTOR_BIN_1, OUTPUT);
		pinMode(PIN_MOTOR_STBY, OUTPUT);
	}

	void DeviceDriver_Motor::setMotorControl(bool leftDirection, uint8_t leftSpeed, bool rightDirection, uint8_t rightSpeed)
	{
		digitalWrite(PIN_MOTOR_STBY, HIGH);

		switch (leftDirection)
		{
		case DIR_FORWARD:
			digitalWrite(PIN_MOTOR_AIN_1, HIGH);
			analogWrite(PIN_MOTOR_PWMA, leftSpeed);
			break;

		case DIR_BACKWARD:
			digitalWrite(PIN_MOTOR_AIN_1, LOW);
			analogWrite(PIN_MOTOR_PWMA, leftSpeed);
			break;

		default:
			analogWrite(PIN_MOTOR_PWMA, 0);
			digitalWrite(PIN_MOTOR_STBY, LOW);
			break;
		}

		switch (rightDirection)
		{
		case DIR_FORWARD:
			digitalWrite(PIN_MOTOR_BIN_1, HIGH);
			analogWrite(PIN_MOTOR_PWMB, rightSpeed);
			break;

		case DIR_BACKWARD:
			digitalWrite(PIN_MOTOR_BIN_1, LOW);
			analogWrite(PIN_MOTOR_PWMB, rightSpeed);
			break;

		default:
			analogWrite(PIN_MOTOR_PWMB, 0);
			digitalWrite(PIN_MOTOR_STBY, LOW);
			break;
		}
	}
}