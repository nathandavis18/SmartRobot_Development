#include <WiFi.h>
#include <cmath>
#include "MessageSerializer.h"
#include "SmartRobotDtos.h"
#include "String.hpp"
#include "ESP32Board.hpp"

namespace ESP32Board
{
#define RXD2 3
#define TXD2 40
	constexpr char ssid[] = "SR_Test";
	constexpr char pass[] = "srtest1234";

	WiFiUDP udp;
	const IPAddress serverIp(192, 168, 137, 1);
	constexpr int serverPort = 1818;
	constexpr int localPort = 100;

	bool connected = false;
	bool standby = false;

	double lastX = 0, lastY = 0; //The last x and y positions the robot needs to go to before the assignment is finished
	sr::Attitude attitude{};

	sr::Position position{};
	sr::Velocity velocity{};

	sr::RobotMessageData robotMsgData;
	sr::MyVariant obj;

	sr::DefaultString message, receiveBuff;

	void connectToWifi(void)
	{
		if (WiFi.status() == WL_CONNECTED) return;

		WiFi.begin(ssid, pass);
		delay(200);
		while (WiFi.status() != WL_CONNECTED);
	}

	void startConnectionToServer(void)
	{
		udp.begin(localPort);

		delay(100);

		sr::SmartRobotAsset asset;
		sr::serialize<sr::SmartRobotAsset>(asset, message);

		while (!connected)
		{
			udp.beginPacket(serverIp, serverPort);
			udp.print(message.c_str());
			udp.endPacket();

			delay(300);
			udp.parsePacket();
			if (udp.available())
			{
				udp.readString();
				connected = true;
			}
		}

		message.clear();
	}

	void handleMessageFromRobot(void)
	{
		bool success = sr::deserializeRobotMessage(receiveBuff, robotMsgData);

		if (!success) return;

		switch (robotMsgData.type)
		{
		case sr::MsgFromRobotType::Standby:
			Serial.println(F("Received Standby"));
			standby = true;
			position.x = lastX;
			position.y = lastY;

			velocity.x = 0;
			velocity.y = 0;
			break;

		case sr::MsgFromRobotType::Distance:
			Serial.println(F("Received Distance"));
			position.x += robotMsgData.distance * std::cos(attitude.yaw);
			position.y += robotMsgData.distance * std::sin(attitude.yaw);
			break;

		default:
			break;
		}

		receiveBuff.clear();
		robotMsgData.clear();
	}

	void receiveFromRobot(void)
	{
		int x = Serial2.available();

		char c = '\0';
		while (x)
		{
			--x;
			c = (char)Serial2.read();
			if (c != '\n' && c != '\r' && c != '\x1b')
				receiveBuff += c;

			if (c == '\x1b' && receiveBuff.length())
			{
				Serial.print("Received from robot: "); Serial.println(receiveBuff.c_str());
				handleMessageFromRobot();
				receiveBuff.clear();
				robotMsgData.clear();
				continue;
			}
		}

		receiveBuff.clear();
		robotMsgData.clear();
	}

	void sendAssignmentToRobot(void)
	{
		if (obj.alternative != sr::MyVariant::alternative_t::pathassignment)
			return;

		double vel, heading, xDiff, yDiff, distance;
		double prevX = position.x, prevY = position.y;

		for (uint8_t i = 0; i < obj.value.pa.waypoints.size(); ++i)
		{
			vel = obj.value.pa.waypoints[i].desiredVelocity;
			heading = obj.value.pa.waypoints[i].heading;
			xDiff = obj.value.pa.waypoints[i].point.x - prevX;
			yDiff = obj.value.pa.waypoints[i].point.y - prevY;
			prevX = obj.value.pa.waypoints[i].point.x;
			prevY = obj.value.pa.waypoints[i].point.y;
			distance = std::sqrt((xDiff * xDiff) + (yDiff * yDiff));
			message += "{\"v\":";
			message.concat(vel);

			message += ",\"d\":";
			message.concat(distance);

			message += ",\"h\":";
			message.concat(heading);

			message.concat('}');

			if (i < obj.value.pa.waypoints.size() - 1)
				message.concat('~');
			else
				message.concat('\x1b');
		}

		lastX = obj.value.pa.waypoints.last().point.x;
		lastY = obj.value.pa.waypoints.last().point.y;

		Serial.print("Sending to robot: "); Serial.println(message.substring<sr::DefaultString>(0, message.length() - 1).c_str());
		Serial2.print(message.c_str());

		message.clear();

		attitude.yaw = obj.value.pa.waypoints.last().heading;
		velocity.x = fabs(vel);
		standby = false;
	}

	void sendReceivedToServer(void)
	{
		if (obj.alternative != sr::MyVariant::alternative_t::pathassignment)
			return;

		message = "{Smart Robot,AssignmentReceived}";
		message += (int)obj.value.pa.pathID;

		int x = 0;
		delay(500);
		while (!x)
		{
			udp.beginPacket(serverIp, serverPort);
			udp.print(message.c_str());
			udp.endPacket();

			delay(100);

			udp.parsePacket();
			x = udp.available();
		}

		message.clear();
	}

	void handleNewAssignment(void)
	{
		sendReceivedToServer();
		sendAssignmentToRobot();
		obj.alternative = sr::MyVariant::alternative_t::none;
	}

	int udpAvailable = 0;
	void handleUdpPacket(void)
	{
		udp.parsePacket();

		udpAvailable = udp.available();
		if (!udpAvailable) return;

		char c = '\0';
		while (udpAvailable)
		{
			c = (char)udp.read();
			if (c != '\x1b' && c != '\n' && c != '\r')
				receiveBuff += c;
			--udpAvailable;
		}

		if (receiveBuff.length())
		{
			Serial.println(receiveBuff.c_str());
			sr::deserialize(receiveBuff, obj);

			switch (obj.alternative)
			{
			case sr::MyVariant::alternative_t::pathassignment:
				handleNewAssignment();
				break;

			case sr::MyVariant::alternative_t::none:
			default:
				break;
			}
		}

		receiveBuff.clear();
	}

	unsigned long lastStandbyInterval = 0;
	unsigned long standbyInterval = 500;

	constexpr uint8_t dtoInterval = 100;
	unsigned long prevDtoInterval = 0;

	enum class DtoToSend
	{
		Position,
		Attitude,
		Velocity
	};

	DtoToSend dtoToSend = DtoToSend::Position;

	void sendDtos(void)
	{
		unsigned long currentInterval = millis();

		if (standby && currentInterval - lastStandbyInterval >= standbyInterval)
		{
			message = "{Smart Robot,StandbyMode}";
			udp.beginPacket(serverIp, serverPort);
			udp.print(message.c_str());
			udp.endPacket();

			message.clear();
			delay(50);

			lastStandbyInterval = millis();
		}

		currentInterval = millis();
		if (currentInterval - prevDtoInterval < dtoInterval) return;

		prevDtoInterval = currentInterval;

		switch (dtoToSend)
		{
		case DtoToSend::Position:
			//Serial.println(F("Sending Position"));
			sr::serialize<sr::Position>(position, message);
			dtoToSend = DtoToSend::Attitude;
			break;

		case DtoToSend::Attitude:
			//Serial.println(F("Sending Attitude"));
			sr::serialize<sr::Attitude>(attitude, message);
			dtoToSend = DtoToSend::Velocity;
			break;

		case DtoToSend::Velocity:
			//Serial.println(F("Sending Velocity"));
			sr::serialize<sr::Velocity>(velocity, message);
			dtoToSend = DtoToSend::Position;
			break;
		}

		delay(50);

		udp.beginPacket(serverIp, serverPort);
		udp.print(message.c_str());
		udp.endPacket();

		message.clear();
	}

	void setup()
	{
		Serial.begin(9600);
		Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

		delay(500);

		connectToWifi();

		startConnectionToServer();
	}

	void loop(void)
	{
		if (WiFi.status() != WL_CONNECTED || !connected)
		{
			udp.stop();
			connected = false;
			connectToWifi();
			startConnectionToServer();
			return;
		}

		//Serial.println(F("Here we go again"));
		sendDtos();
		handleUdpPacket();
		receiveFromRobot();
	}
}