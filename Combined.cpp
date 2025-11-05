#include <thread>
#include "SmartRobot_Uno_Rework/SmartRobot_Rework_Uno.hpp"
#include "SmartRobot_ESP_Rework/SmartRobot_Rework_ESP.hpp"

int main() 
{
	std::thread t1(startEspBoard);
	std::thread t2(startUnoBoard);
	t1.join();
	t2.join();
}