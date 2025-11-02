#pragma once
#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <array>
#include <MinimalSocket/udp/UdpSocket.h>
#define F(x) x

constexpr bool WL_CONNECTED = 1;
constexpr int SERIAL_8N1 = 0;

struct IPAddress
{
public:
	IPAddress(int a, int b, int c, int d) : octet1(a), octet2(b), octet3(c), octet4(d) {}
	IPAddress operator=(const IPAddress& other) 
	{
		octet1 = other.octet1;
		octet2 = other.octet2;
		octet3 = other.octet3;
		octet4 = other.octet4;
		return *this;
	}
	const char* toString() const 
	{
		static char buffer[16];
		std::snprintf(buffer, sizeof(buffer), "%d.%d.%d.%d", octet1, octet2, octet3, octet4);
		return buffer;
	}
private:
	int octet1, octet2, octet3, octet4;
};

class WiFiUDP
{
public:
	void begin(int port) 
	{
		_socket = std::make_unique<MinimalSocket::udp::Udp<true>>(port, MinimalSocket::AddressFamily::IP_V4);
		_socket->open();
	}
	void stop() 
	{
		_socket->shutDown();
	}
	void beginPacket(const IPAddress& ip, int port) 
	{
		if (_remoteAddress != nullptr) return;
		auto ip1 = IPAddress(127, 0, 0, 1);
		_remoteAddress = std::make_unique<MinimalSocket::Address>(ip1.toString(), static_cast<MinimalSocket::Port>(port));
	}
	void print(const char* message) 
	{
		_buffer = message;
	}
	void endPacket()
	{
		_socket->sendTo(_buffer, *_remoteAddress);
		_buffer.clear();
	}
	void parsePacket() 
	{
		const MinimalSocket::Timeout timeout = MinimalSocket::Timeout(500);
		auto buffer = _socket->receive(1000, timeout);
		if (buffer.has_value())
		{
			_buffer = buffer.value().received_message.c_str();
		}
	}
	int available() 
	{ 
		return _buffer.length();
	}
	std::string readString() 
	{
		std::string tmp = _buffer;
		_buffer.clear();
		return tmp;
	}
	char read() 
	{
		if (_buffer.empty()) return '\0';
		char ch = _buffer[0];
		_buffer.erase(0, 1);
		return ch;
	}

private:
	std::unique_ptr<MinimalSocket::udp::Udp<true>> _socket;
	std::unique_ptr<MinimalSocket::Address> _remoteAddress;
	std::string _buffer;
};

namespace details
{
	class WiFiStuff
	{
	public:
		bool status() 
		{ 
			return connected;
		}
		void begin(const char* ssid, const char* pass) 
		{
			connected = true;
		}
	private:
		bool connected = false;
	};

	class SerialImpl
	{
	public:
		static void println(const char* message)
		{
			std::cout << message << std::endl;
		}
		static void print(const char* message)
		{
			std::cout << message;
		}
		void begin(int rate, int config = 0, int rxPin = 0, int txPin = 0) {}
		int available() { return 0; }
		char read() { return '\0'; }
	};
}

void delay(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

const long long millis()
{
	auto now = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
	return duration.count();
}