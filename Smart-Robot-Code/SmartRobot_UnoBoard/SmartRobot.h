#pragma once
#include <Arduino.h>
#include "DeviceDrivers.h"

class SmartRobot
{
public:
	struct Command
	{
		float velocity, distance, heading;
		Command(float v, float d, float h) : velocity(v), distance(d), heading(h) {}
		Command() : velocity(0), distance(0), heading(0) {}
	};
public:
	SmartRobot(void);

	void SmartRobotInit();

	void updateMotion(float speed, float distance);
	void updateAngle(float newHeading);
	void getSerialData(void);
	void updateDistanceData(void);
	void updateDistanceLeft(void);
	void sendDistanceMoved(bool forced = false);
	void startCommand(void);

public:
	bool isMoving = false;

private:
	enum SmartRobotMode
	{
		Standby,
		Moving,
		Estop,
		Unknown
	};

private:
	void updateRobotAngle(bool leftDirection, bool rightDirection, float headingToFace);
	void moveRobot(uint8_t speed, bool forward = true);
	void stopRobot();

private:
	float _currentHeading;
	SmartRobotMode _mode;
	DeviceDriver_Motor _motorControl;
	DeviceDriver_Voltage _voltageControl;
	DeviceDriver_Key _keyControl;

	static constexpr float angleEpsilon = 10;
};