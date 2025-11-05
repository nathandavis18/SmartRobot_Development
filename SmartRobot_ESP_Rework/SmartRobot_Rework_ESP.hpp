#pragma once
#include <Arduino.h>

namespace ESP32Board
{
	inline static details::SerialImpl Serial(0, 0, true);
	inline static details::SerialImpl Serial2(105, 106, false);
}

void startEspBoard();