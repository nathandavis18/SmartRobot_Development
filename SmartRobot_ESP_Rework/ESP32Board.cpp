#include <WiFi.h>
#include <cmath>
#include "MessageSerializer.h"
#include "SmartRobotDtos.h"
#include "String.hpp"
#include "ESP32Board.hpp"
#include "SmartRobot_Rework_ESP.hpp"


namespace ESP32Board 
{
	// Some networking stuff
	constexpr char ssid[] = "SR_Test";
	constexpr char pass[] = "srtest1234";
	WiFiUDP udpSocket;
	const IPAddress serverIp(192, 168, 137, 1);
	constexpr int serverPort = 1818;
	constexpr int localPort = 100;

	// Connection status
	bool connected = false;
	constexpr unsigned int connectionTimeoutMs = 5000;
	unsigned long lastMessageTime = 0;

	// DTOs to send to Mobius
	sr::Attitude attitude{};
	sr::Position position{};
	sr::Velocity velocity{};
	enum class MessageToSend
	{
		AttitudeMsg,
		PositionMsg,
		VelocityMsg
	};
	MessageToSend messageToSend = MessageToSend::AttitudeMsg;
	double finalX = 0, finalY = 0;

	// Some stuff for communicating with the Uno board
	constexpr uint8_t RXD2 = 3;
	constexpr uint8_t TXD2 = 40;
	sr::RobotMessageData robotMsgData;

	// A custom variant to parse incoming messages
	sr::MyVariant variant;

	// Strings to receive incoming messages and send messages
	sr::DefaultString udpReceiveBuff, udpSendBuff;
	sr::DefaultString serialReceiveBuff, serialSendBuff;

	// Start the WiFi connection to communicate with Mobius
	void connect_to_wifi() 
	{
		if (WiFi.status() == WL_CONNECTED) return;
		WiFi.begin(ssid, pass);
		while(WiFi.status() != WL_CONNECTED) 
			delay(50);
	}

	void send_message_to_mobius()
	{
		udpSocket.beginPacket(serverIp, serverPort);
		udpSocket.print(udpSendBuff.c_str());
		udpSocket.endPacket();
		udpSendBuff.clear();
	}

	void send_message_to_robot()
	{
		Serial2.print(serialSendBuff.c_str());
		serialSendBuff.clear();
	}

	void send_asset_connection_message()
	{
		sr::SmartRobotAsset asset;
		sr::serialize<sr::SmartRobotAsset>(asset, udpSendBuff);

		do
		{
			udp.beginPacket(serverIp, serverPort);
			udp.print(udpSendBuff.c_str());
			udp.endPacket();

			delay(300);
			udp.parsePacket();
			if (udp.available())
			{
				udp.readString();
				connected = true;
			}
		} while (!connected);

		udpSendBuff.clear();
		lastMessageTime = millis();
	}

	// Start all of the serial communications and connect to Mobius server
	void setup()
	{
		Serial.begin(9600);
		Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
		connect_to_wifi();
		udpSocket.begin(localPort);
		send_asset_connection_message();
	}

	void send_dtos()
	{
		switch (messageToSend)
		{
			case MessageToSend::AttitudeMsg:
				sr::serialize<sr::Attitude>(attitude, udpSendBuff);
				messageToSend = MessageToSend::PositionMsg;
				break;
			case MessageToSend::PositionMsg:
				sr::serialize<sr::Position>(position, udpSendBuff);
				messageToSend = MessageToSend::VelocityMsg;
				break;
			case MessageToSend::VelocityMsg:
				sr::serialize<sr::Velocity>(velocity, udpSendBuff);
				messageToSend = MessageToSend::AttitudeMsg;
				break;
		}
		send_message_to_mobius();
	}

	// TODO: Finish implementing the new path assignment handler
	void handle_incoming_path_assignment(const sr::PathAssignment& pa)
	{
		if (pa.waypoints.size() == 0)
			return;
		finalX = pa.waypoints.last().x;
		finalY = pa.waypoints.last().y;
		// Send waypoints to robot here
	}

	void read_single_message_from_mobius()
	{
		udpSocket.parsePacket();
		int bytesAvailable = udpSocket.available();
		if (bytesAvailable)
		{
			char c = 0;
			while (bytesAvailable--) 
			{
				c = udpSocket.read();
				if (c == '\x1b')
					break;
				udpReceiveBuff += c;
			}

			sr::deserialize(udpReceiveBuff, variant);
			switch(variant.alternative)
			{
				case sr::MyVariant::alternative_t::pathassignment:
					handle_incoming_path_assignment(variant.value.pa);
					break;
				case sr::MyVariant::alternative_t::stop:
					serialSendBuff = "{stop}\x1b";
					send_message_to_robot();
					break;
				default:
					break;
			}
			lastMessageTime = millis();
		}
	}

	void read_single_message_from_robot()
	{
		int bytesAvailable = Serial2.available();
		if (bytesAvailable)
		{
			char c = 0;
			while(bytesAvailable--)
			{
				c = Serial2.read();
				if (c == '\x1b')
					break;
				serialReceiveBuff += c;
			}
			bool success = sr::deserializeRobotMessage(serialReceiveBuff, robotMsgData);
			switch (robotMsgData.type)
			{
			case sr::MsgFromRobotType::Distance:
				// Handle distance message
				break;
			case sr::MsgFromRobotType::Standby:
				// Handle standby message
				break;
			default:
				break;
			}
		}
	}

	// Main loop
	void loop()
	{
		if(millis() - lastMessageTime > connectionTimeoutMs)
			connected = false;
		if(WiFi.status() != WL_CONNECTED)
			connect_to_wifi();
		if(!connected)
			send_asset_connection_message();

		// Now that all the checks are done, we can handle communication
		send_dtos();
		read_single_message_from_mobius();
		// TODO: Finish other communication things
	}
}

namespace ESP32Board
{
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
			Serial.println(F("Sending Standby Mode"));
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