#include "SmartRobot.h"
#include "../lib/Array.hpp"
#include "../lib/String.hpp"
#include "../lib/JsonDict.hpp"
#include "../../SmartRobot_Uno_Rework/UnoBoard.hpp"
#include "../../SmartRobot_Uno_Rework/SmartRobot_Rework_Uno.hpp"

namespace helpers
{
	float get_degrees(float radians)
	{
		return radians * 180 / 3.14;
	}

	float get_radians(float degrees)
	{
		return degrees * 3.14 / 180;
	}

	bool should_spin_clockwise(float currentHeading, float newHeading)
	{
		if (currentHeading <= 0)
		{
			if (newHeading < 0)
				return newHeading < currentHeading;
			return newHeading > (currentHeading + 180);
		}

		if (newHeading <= 0)
		{
			return newHeading > (currentHeading - 180);
		}

		return newHeading < currentHeading;
	}

	int get_rotation_time(float currentHeading, float newHeading)
	{
		static constexpr float timePerDegree = 60.0 / 9.0;
		if((currentHeading < 0 && newHeading < 0) || (currentHeading >= 0 && newHeading >= 0))
		{
			return static_cast<int>(fabs(currentHeading - newHeading) * timePerDegree);
		}

		if (currentHeading < 0)
		{
			if (should_spin_clockwise(currentHeading, newHeading))
			{
				float rotationAmount = 180 - fabs(currentHeading);
				return static_cast<int>((180 - (newHeading - rotationAmount)) * timePerDegree);
			}
			return static_cast<int>((newHeading + fabs(currentHeading)) * timePerDegree);
		}

		if (should_spin_clockwise(currentHeading, newHeading))
		{
			return static_cast<int>(fabs(newHeading - currentHeading) * timePerDegree);
		}
		float rotationAmount = 180 - currentHeading;
		return static_cast<int>((180 - fabs(newHeading + rotationAmount)) * timePerDegree);
	}

	bool angle_is_same(float currentHeading, float newHeading, float epsilon)
	{
		return fabs(currentHeading - newHeading) <= epsilon;
	}
}
namespace sr
{
	using namespace UnoBoard;

	// These are the string buffers to use for different things. Pre-allocated memory, so no memory overflows
	TinyString sendBuffer;
	DefaultString receiveBuffer;
	JsonString splitDataBuffer;

	// The array of segments to execute
	uint8_t currentSegmentIndex = 0;
	MyArray<SmartRobot::Command, 15> segments;

	// Some timer variables to track distance updates
	unsigned long lastDistanceUpdateTime = 0;
	unsigned long currentInterval;
	unsigned long timeDiff;

	void send_message()
	{
		sendBuffer.concat('\x1b');
		Serial.print(sendBuffer.c_str());
		sendBuffer.clear();
	}

	void send_standby_mode()
	{
		sendBuffer = "{Smart Robot,Standby}";
		send_message();
		delay(50);
	}

	SmartRobot::SmartRobot() : _mode(SmartRobotMode::Standby), _currentHeading(0), _currentVelocity(0) {}

	void SmartRobot::SmartRobotInit()
	{
		Serial.begin(115200);
		_voltageControl.VoltageInit();
		_motorControl.MotorInit();
		_keyControl.KeyInit();
	}

	void SmartRobot::handle_incoming_data()
	{
		if (_mode == SmartRobotMode::Unknown || _mode == SmartRobotMode::Estop) return;

		char c = 0;
		int x = Serial.available();
		while (x--)
		{
			c = Serial.read();
			if (c == '\x1b')
			{
				send_message();
				break;
			}

			if (c != '\n' && c != ' ' && c != '\t' && c != '\r')
				receiveBuffer += c;
		}

		if (c == '\x1b' && receiveBuffer.char_at(0) == '{')
		{
			segments.clear();

			int strIndex = 0;
			MyDictionary dict;

			while(strIndex < receiveBuffer.length())
			{
				while (strIndex < receiveBuffer.length() && receiveBuffer.char_at(strIndex) != '~')
				{
					splitDataBuffer += receiveBuffer.char_at(strIndex);
					++strIndex;
				}

				parse_json(splitDataBuffer, dict);

				Command currentSegment;
				currentSegment.velocity = dict["v"];
				currentSegment.distance = dict["d"];
				currentSegment.heading = helpers::get_degrees(dict["h"]);
				
				dict.clear();
				splitDataBuffer.clear();

				segments.insert_back(currentSegment);
				++strIndex;
			}

			receiveBuffer.clear();
			send_message();

			currentSegmentIndex = 0;
			startCommand();
		}
		else if (receiveBuffer.length() && receiveBuffer.char_at(0) != '{') 
			receiveBuffer.clear();
	}

	void SmartRobot::startCommand()
	{
		if (currentSegmentIndex >= segments.size()) return;

		_currentVelocity = fabs(segments.at(currentSegmentIndex).velocity);
		updateAngle(segments.at(currentSegmentIndex).heading);

		lastDistanceUpdateTime = millis();
		if (segments.at(currentSegmentIndex).distance > 0)
			updateMotion(_currentVelocity);

		isMoving = true;
	}

	void SmartRobot::updateDistanceData()
	{
		delay(5);
		if (segments.at(currentSegmentIndex).distance <= 0)
		{
			if (++currentSegmentIndex < segments.size())
			{
				startCommand();
			}
			else
			{
				stopRobot();
				delay(10);
				_mode = SmartRobotMode::Standby;
				delay(50);
				send_standby_mode();
				segments.clear();
				currentSegmentIndex = 0;
				isMoving = false;
			}
			return;
		}

		currentInterval = millis();
		timeDiff = currentInterval - lastDistanceUpdateTime;

		const float distanceMoved = _currentVelocity * (timeDiff / 1000.0) * 10;
		segments.at(currentSegmentIndex).distance -= distanceMoved;
		lastDistanceUpdateTime = currentInterval;

		sendDistanceMoved(distanceMoved);
	}

	void SmartRobot::sendDistanceMoved(const float distance)
	{
		sendBuffer = "{Smart Robot,Distance}";
		sendBuffer.concat(distance);
		sendBuffer.concat(',');
		sendBuffer.concat(_currentVelocity);
		sendBuffer.concat(',');
		sendBuffer.concat(helpers::get_radians(_currentHeading));
		send_message();
	}

	void SmartRobot::updateAngle(float newHeading)
	{
		if (helpers::angle_is_same(_currentHeading, newHeading, angleEpsilon))
			return;
		if (helpers::should_spin_clockwise(_currentHeading, newHeading))
		{
			updateRobotAngle(false, true, newHeading);
		}
		else
		{
			updateRobotAngle(true, false, newHeading);
		}
		_currentHeading = newHeading;
	}

	void SmartRobot::updateMotion(float speed)
	{
		uint16_t speedUnits = speed * 35;
		if (speedUnits > 255) speedUnits = 255;

		moveRobot(speedUnits);
	}

	void SmartRobot::updateRobotAngle(bool rightDirection, bool leftDirection, float headingToFace)
	{
		int time = helpers::get_rotation_time(_currentHeading, headingToFace);
		_motorControl.setMotorControl(rightDirection, 100, leftDirection, 100);

		long curTime = millis();
		while (millis() - curTime < time);

		stopRobot();
		delay(50);
	}

	void SmartRobot::moveRobot(uint8_t speed, bool forward)
	{
		_motorControl.setMotorControl(forward, speed, forward, speed);
	}

	void SmartRobot::stopRobot()
	{
		_motorControl.setMotorControl(false, 0, false, 0);
	}
}