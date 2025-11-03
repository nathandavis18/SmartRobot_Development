#pragma once
#include <stdint.h>

namespace sr
{
	class DeviceDriver_Key
	{
	public:
		void KeyInit(void);

#define PIN_KEY 2
#define KEYVALUE_MAX 4

		static uint8_t keyValue;
	};

	class DeviceDriver_Voltage
	{
	public:
		void VoltageInit(void);
		float getAnalogue(void);

	private:
#define PIN_A3  (17)
		static constexpr uint8_t A3 = PIN_A3;
#define PIN_VOLTAGE A3
	};

	class DeviceDriver_Motor
	{
	public:
		void MotorInit(void);

		// For the direction, false means reverse, true means forward
		void setMotorControl(bool leftDirection, uint8_t leftSpeed, bool rightDirection, uint8_t rightSpeed);

	private:
#define PIN_MOTOR_PWMA 5
#define PIN_MOTOR_PWMB 6

#define PIN_MOTOR_AIN_1 7
#define PIN_MOTOR_BIN_1 8

#define PIN_MOTOR_STBY 3

	public:
#define SPEED_MAX 255
#define DIR_FORWARD true
#define DIR_BACKWARD false
#define DIRECTION_VOID 3
	};
}