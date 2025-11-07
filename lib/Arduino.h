#pragma once
#include <stdint.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <memory>
#include <array>
#include <Arduino.h>
#include <MinimalSocket/udp/UdpSocket.h>

// Wifi stuff
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
		try
		{
			const MinimalSocket::Timeout timeout = MinimalSocket::Timeout(50);
			auto buffer = _socket->receive(1000, timeout);
			if (buffer.has_value())
			{
				_buffer = buffer.value().received_message.c_str();
			}
		}
		catch (...) {}
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
}

static details::WiFiStuff WiFi;


// Other stuff ==================================================

#define BUFFER_LENGTH 32
#define F(x) x

namespace timestuff
{
	static std::chrono::steady_clock::time_point startTime;
}

inline const long long millis()
{
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timestuff::startTime);
	return duration.count();
}

inline void delay(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline uint8_t min(uint8_t a, uint8_t b)
{
	return (a < b) ? a : b;
}

namespace details
{
	class WireImpl
	{
	public:
		void beginTransmission(uint8_t address) {}
		void send(uint8_t data) {}
		void endTransmission() {}
		void requestFrom(uint8_t address, int length) {}
		bool available() { return false; }
		uint8_t receive() { return 0; }
	};

	class SerialImpl
	{
	public:
		SerialImpl(int thisPort, int connPort, bool printToConsole) : _thisPort(thisPort), _connectionPort(connPort), _udp(WiFiUDP()), _printToConsole(printToConsole)
		{}

		void println(const std::string& message)
		{
			print(message + "\n");
		}

		void print(const std::string& message)
		{
			if (_printToConsole) 
			{
				std::cout << message;
			}
			if(_connectionPort != 0)
			{
				IPAddress ip(127, 0, 0, 1);
				_udp.beginPacket(ip, _connectionPort);
				_udp.print(message.c_str());
				_udp.endPacket();
			}
		}
		void begin(int rate, int config = 0, int rxPin = 0, int txPin = 0)
		{
			if (_thisPort != 0)
				_udp.begin(_thisPort);
		}
		int available()
		{
			_udp.parsePacket();
			return _udp.available();
		}
		char read()
		{
			return _udp.read();
		}

	private:
		bool _printToConsole;
		int _thisPort;
		int _connectionPort;
		WiFiUDP _udp;
	};
}

static details::WireImpl Wire;

namespace setupStuff
{
	inline void setup()
	{
		timestuff::startTime = std::chrono::steady_clock::now();
	}
}