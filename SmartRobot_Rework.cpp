#include <thread>
#include "ESP32Board.hpp"
#include "UnoBoard.hpp"
#include <Arduino.h>

void handleUnoBoard()
{
    UnoBoard::setup();
    while (true)
    {
        UnoBoard::loop();
    }
}
void handleEspBoard()
{
	ESP32Board::setup();
    while (true)
    {
        ESP32Board::loop();
	}
}

int main()
{
    setupStuff::setup();

    std::thread t1(handleUnoBoard);
    std::thread t2(handleEspBoard);

    t1.join();
    t2.join();
}