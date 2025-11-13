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

	// These are the final X and Y positions from the waypoint assignment, just to align the robot with the path since it can't track its own position
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
		serialSendBuff.concat('\x1b');
		Serial.print(F("Sending to robot: ")); Serial.println(serialSendBuff.substring<sr::DefaultString>(0, serialSendBuff.length() - 1).c_str());

		Serial2.print(serialSendBuff.c_str());
		serialSendBuff.clear();
	}

	void send_asset_connection_message()
	{
		sr::SmartRobotAsset asset;
		sr::serialize<sr::SmartRobotAsset>(asset, udpSendBuff);

		Serial.println(F("Sending asset connection message"));

		send_message_to_mobius();
		delay(300);

		udpSocket.parsePacket();
		if (udpSocket.available())
		{
			udpSocket.readString();
			connected = true;
			lastMessageTime = millis();
		}

	}

	static unsigned long timeSinceLastSend = 0;
	constexpr uint8_t minimumTimeBetweenSendMs = 100;
	void send_dtos()
	{
		if (millis() - timeSinceLastSend < minimumTimeBetweenSendMs)
			return;
		timeSinceLastSend = millis();

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

	void handle_incoming_path_assignment(const sr::PathAssignment& pa)
	{
		if (pa.waypoints.size() == 0)
			return;

		finalX = pa.waypoints.last().point.x;
		finalY = pa.waypoints.last().point.y;

		double vel, heading, distance;
		double prevX = position.x, prevY = position.y;

		for(int i = 0; i < pa.waypoints.size(); ++i)
		{
			const sr::Waypoint& wp = pa.waypoints.at(i);
			vel = wp.desiredVelocity;
			heading = wp.heading;
			distance = std::sqrt(std::pow(wp.point.x - prevX, 2) + std::pow(wp.point.y - prevY, 2));
			prevX = wp.point.x;
			prevY = wp.point.y;
			
			serialSendBuff.concat("{\"v\":");
			serialSendBuff.concat(vel);
			serialSendBuff.concat(",\"h\":");
			serialSendBuff.concat(heading);
			serialSendBuff.concat(",\"d\":");
			serialSendBuff.concat(distance);
			serialSendBuff.concat("}");

			if(i < pa.waypoints.size() - 1)
				serialSendBuff.concat("~");
		}

		send_message_to_robot();
	}

	void ack_path_assignment(const sr::PathAssignment& pa)
	{
		if (pa.waypoints.size() == 0)
			return;

		Serial.println(F("Sending path assignment Ack"));
		udpSendBuff = "{Smart Robot,AssignmentReceived}";
		udpSendBuff.concat(static_cast<int>(pa.pathID));
		send_message_to_mobius();
	}

	void send_stop_to_robot()
	{
		Serial.println(F("Stopping robot"));
		serialSendBuff = "{stop}";
		send_message_to_robot();
	}

	void handle_teleop_command(const sr::TeleopCommand& cmd)
	{
		Serial.println(F("Handling teleop command"));
		Serial.println(std::to_string(cmd.velocity) + "," + std::to_string(cmd.turnRate));
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
					ack_path_assignment(variant.value.pa);
					handle_incoming_path_assignment(variant.value.pa);
					break;
				case sr::MyVariant::alternative_t::stop:
					send_stop_to_robot();
					break;
				case sr::MyVariant::alternative_t::teleop:
					handle_teleop_command(variant.value.tc);
					break;
				default:
					break;
			}
			lastMessageTime = millis();
			udpReceiveBuff.clear();
			variant.clear();
		}
	}

	void handle_distance_message()
	{
		attitude.yaw = robotMsgData.heading;
		velocity.x = robotMsgData.velocity;
		position.x += robotMsgData.distance * std::cos(attitude.yaw);
		position.y += robotMsgData.distance * std::sin(attitude.yaw);
		Serial.println(F("Distance message received from robot"));
	}

	void handle_standby_message()
	{
		Serial.println(F("Standby message received from robot"));
		udpSendBuff = "{Smart Robot,StandbyMode}";
		send_message_to_mobius();

		position.x = finalX;
		position.y = finalY;
		velocity.x = 0;
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
			if (sr::deserializeRobotMessage(serialReceiveBuff, robotMsgData))
			{
				switch (robotMsgData.type)
				{
				case sr::MsgFromRobotType::Distance:
					handle_distance_message();
					break;
				case sr::MsgFromRobotType::Standby:
					handle_standby_message();
					break;
				default:
					break;
				}
			}
		}
		serialReceiveBuff.clear();
	}

	// Start all of the serial communications and connect to Mobius server
	void setup()
	{
		Serial.begin(9600);
		Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
		connect_to_wifi();
		udpSocket.begin(localPort);
		lastMessageTime = millis();
	}

	// Main loop
	void loop()
	{
		if(WiFi.status() != WL_CONNECTED)
			connect_to_wifi();
		if (millis() - lastMessageTime > connectionTimeoutMs) 
		{
			connected = false;
			send_asset_connection_message();
		}
		if (!connected)
			return;

		// Now that all the checks are done, we can handle communication
		send_dtos();
		read_single_message_from_mobius();
		read_single_message_from_robot();
	}
}