#include "UnoBoard.hpp"
#include "SmartRobot_Rework_Uno.hpp"

void startUnoBoard()
{
	setupStuff::setup();
	UnoBoard::setup();
	while (true)
	{
		UnoBoard::loop();
	}
}