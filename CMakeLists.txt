cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
set(BUILD_MinimalCppSocket_SAMPLES OFF CACHE BOOL "" FORCE) # you don't want the samples in this case
FetchContent_Declare(
min_sock
GIT_REPOSITORY https://github.com/andreacasalino/Minimal-Socket
GIT_TAG        master
)
FetchContent_MakeAvailable(min_sock)

project(SmartRobot_Combined)
# --- Your executable ---

add_executable(SmartRobot_Combined 
	"Combined.cpp"
    "SmartRobot_ESP_Rework/SmartRobot_Rework_ESP.cpp"
    "SmartRobot_ESP_Rework/ESP32Board.cpp" "SmartRobot_ESP_Rework/ESP32Board.hpp"
    "Smart-Robot-Code/SmartRobot_ESP32Board/MessageSerializer.cpp"
	"lib/Arduino.h"

	"SmartRobot_Uno_Rework/SmartRobot_Rework_Uno.cpp"
	"SmartRobot_Uno_Rework/UnoBoard.cpp" "SmartRobot_Uno_Rework/UnoBoard.hpp"
	"Smart-Robot-Code/SmartRobot_UnoBoard/DeviceDrivers.cpp"
    "Smart-Robot-Code/SmartRobot_UnoBoard/I2Cdev.cpp"
    "Smart-Robot-Code/SmartRobot_UnoBoard/SmartRobot.cpp"
	"lib/avr/wdt.h")

target_include_directories(SmartRobot_Combined PRIVATE lib Smart-Robot-Code/SmartRobot_ESP32Board Smart-Robot-Code/lib Smart-Robot-Code/SmartRobot_UnoBoard)
target_link_libraries(SmartRobot_Combined PRIVATE
    MinimalSocket
)