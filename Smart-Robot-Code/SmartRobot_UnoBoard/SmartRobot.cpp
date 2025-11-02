#include "SmartRobot.h"
#include "ArduinoJson-v6.11.1.h"
#include "../lib/Array.hpp"
#include "../lib/String.hpp"

using ARDUINOJSON_NAMESPACE::StaticJsonDocument;
using MySmallString = details::BasicCustomString<30>;
using MyBigString = details::BasicCustomString<600>;

MySmallString message;
float distanceLeft = 0;
float currentVelocity = 0;
float currentDistanceMoved = 0;
unsigned long lastDistanceInterval = 0;


SmartRobot::SmartRobot(void) : _mode(SmartRobotMode::Standby), _currentHeading(0) {}

struct Command {
	float velocity, distance, heading;
	Command(float v, float d, float h) : velocity(v), distance(d), heading(h) {}
	Command() : velocity(0), distance(0), heading(0) {}
};

sr::MyArray<Command, 15> commands;

void sendStandbyMode(void) {
	Serial.println(F("{Smart Robot,Standby}\x1b"));
	delay(50);
}

void sendReceived(void) {
	Serial.println(F("\x1b"));
	delay(50);
}

float getDegrees(float radians) {
	float degrees = radians * 180 / 3.14;

	while(degrees >= 360.0) degrees -= 360.0;
	while(degrees <= -360.0) degrees += 360.0;

	if(degrees > 180){
		degrees = -180 + (degrees - 180);
	}
	if(degrees < -180){
		degrees = 180 + (degrees + 180);
	}

	return degrees;
}

float getRadians(float degrees){
	return degrees * 3.14 / 180;
}

void SmartRobot::SmartRobotInit() {
	Serial.begin(115200);
	_voltageControl.VoltageInit();
	_motorControl.MotorInit();
	_keyControl.KeyInit();

	_currentHeading = 0;
	_mode = SmartRobotMode::Standby;
}

uint8_t commandsIndex = 0;

MyBigString buffer = "";
String splitData = "";
void SmartRobot::getSerialData() {
	if (_mode == SmartRobotMode::Unknown || _mode == SmartRobotMode::Estop) return;

	char c = "";
	int x = Serial.available();
	while (x) {
		c = Serial.read();
		if (c != '\n' && c != ' ' && c != '\t' && c != '\r')
			buffer += c;
		--x;
		if (c == '\x1b') break;
	}

	static int strIndex = 0;
	static Command currentCommand;
	if (c == '\x1b' && buffer[0] == '{') {
		commands.clear();
		StaticJsonDocument<200> doc;
		while (strIndex < buffer.length()) {
			while (buffer.charAt(strIndex) != '~' && buffer.charAt(strIndex) != '\x1b') {
				splitData += buffer.charAt(strIndex);
				++strIndex;
			}
			deserializeJson(doc, splitData);
			currentCommand.velocity = doc["v"];
			currentCommand.distance = doc["d"];
			currentCommand.heading = getDegrees(doc["h"]);

			doc.clear();
			splitData = "";
			commands.insert(currentCommand);
			++strIndex;
		}

		buffer.clear();
		sendReceived();
		strIndex = 0;
		_mode = SmartRobotMode::Moving;

		commandsIndex = 0;
		startCommand();


	}
	else if (buffer.length() && buffer[0] != '{') buffer.clear();
}

bool awaitingCommand = true;

void SmartRobot::startCommand() {
	if (commandsIndex >= commands.items()) return;

	distanceLeft = fabs(commands[commandsIndex].distance);
	currentVelocity = fabs(commands[commandsIndex].velocity);

	updateAngle(commands.at(commandsIndex).heading);
	lastDistanceInterval = millis();
	if(commands.at(commandsIndex).distance > 0)
		updateMotion(currentVelocity, distanceLeft);

	++commandsIndex;
	isMoving = true;
	awaitingCommand = false;
}

unsigned long currentInterval;
unsigned long timeDiff;
constexpr uint8_t minTimeDiff = 5;

void SmartRobot::updateDistanceData() {
	currentInterval = millis();
	timeDiff = currentInterval - lastDistanceInterval;
	if(timeDiff < minTimeDiff) return;

	currentDistanceMoved = currentVelocity * (timeDiff / 1000.0) * 10;

	lastDistanceInterval = currentInterval;

	updateDistanceLeft();

	if (!awaitingCommand) {
		sendDistanceMoved();
	}
	else {
		if (commandsIndex < commands.items()) {
			stopRobot();
			startCommand();		
		}
		else{
			stopRobot();
			delay(10);
			_mode = SmartRobotMode::Standby;
			delay(50);
			sendStandbyMode();

			distanceLeft = 0;
			currentDistanceMoved = 0;
			commands.clear();
			commandsIndex = 0;
			isMoving = false;
			awaitingCommand = true;
		}
	}
}

unsigned long lastDistanceMovedInterval = 0;
unsigned long distanceMovedInterval = 50;
float distanceMovedSinceLastSend = 0;
unsigned long currentTime;
unsigned long distanceTimeDiff;

void SmartRobot::sendDistanceMoved(bool forced) {
	distanceMovedSinceLastSend += currentDistanceMoved;

	currentTime = millis();
	distanceTimeDiff = currentTime - lastDistanceMovedInterval;
	if (distanceTimeDiff < distanceMovedInterval && !forced) return;


	message = "{Smart Robot,Distance}";
	message.concat(distanceMovedSinceLastSend);
	message.concat('\x1b');
	Serial.println(message);

	message.clear();
	distanceMovedSinceLastSend = 0;
	lastDistanceMovedInterval = millis();
}

void SmartRobot::updateDistanceLeft() {
	distanceLeft -= currentDistanceMoved;

	if (distanceLeft <= 0)
		awaitingCommand = true;
}

bool getRotationDir(float currentHeading, float newHeading) { //True is counter-clockwise
	if (newHeading < 0 && currentHeading > 0)
		return newHeading < (180.0 - currentHeading);

	if (newHeading > 0 && currentHeading < 0)
		return newHeading > (180.0 + currentHeading);

	return newHeading > currentHeading;

}

void SmartRobot::updateAngle(float newHeading) {
	if (_currentHeading == newHeading) return;

	if(fabs(_currentHeading - newHeading) <= angleEpsilon) return;

	if (getRotationDir(_currentHeading, newHeading)) { //Go counter clock-wise
		updateRobotAngle(true, false, newHeading);
	}
	else {
		updateRobotAngle(false, true, newHeading);
	}

	_currentHeading = newHeading;
}

void SmartRobot::updateMotion(float speed, float distance) {
	uint16_t speedUnits = speed * 35;
	if (speedUnits > 255) speedUnits = 255;

	moveRobot(speedUnits);
}

int getTime(float currentHeading, float newHeading){
	if(newHeading < 0 && currentHeading > 0){
		newHeading += 360;
	}
	else if(newHeading > 0 && currentHeading < 0){
		newHeading -= 360.0;
	}

	float diff = fabs(newHeading - currentHeading);

	return diff * 60 / 9;
}

void SmartRobot::updateRobotAngle(bool rightDirection, bool leftDirection, float headingToFace) {
	int time = getTime(_currentHeading, headingToFace);

	_motorControl.setMotorControl(rightDirection, 100, leftDirection, 100);

	long curTime = millis();
	while(millis() - curTime < time) {}

 	stopRobot();
	delay(50);
}

void SmartRobot::moveRobot(uint8_t speed, bool forward) {
	_motorControl.setMotorControl(forward, speed, forward, speed);
}

void SmartRobot::stopRobot() {
	_motorControl.setMotorControl(false, 0, false, 0);
}