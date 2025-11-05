#include "ESP32Board.hpp"
#include "SmartRobot_Rework_ESP.hpp"

void startEspBoard() 
{
	setupStuff::setup();
	ESP32Board::setup();
	while (true)
	{
		ESP32Board::loop();
	}
}