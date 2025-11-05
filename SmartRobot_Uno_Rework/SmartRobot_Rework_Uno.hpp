#pragma once
#include <Arduino.h>
namespace UnoBoard
{
	inline static details::SerialImpl Serial(106, 105, true);
}

void startUnoBoard();