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

project(SmartRobot_Rework_Uno)

add_executable(SmartRobot_Rework_Uno
	"UnoMain.cpp"
	"SmartRobot_Rework_Uno.cpp"
	"UnoBoard.cpp" "UnoBoard.hpp"
	"../Smart-Robot-Code/SmartRobot_UnoBoard/DeviceDrivers.cpp"
    "../Smart-Robot-Code/SmartRobot_UnoBoard/I2Cdev.cpp"
    "../Smart-Robot-Code/SmartRobot_UnoBoard/SmartRobot.cpp"
	"../lib/Arduino.h" "../lib/avr/wdt.h")

target_include_directories(SmartRobot_Rework_Uno PRIVATE ../lib ../Smart-Robot-Code/SmartRobot_UnoBoard ../Smart-Robot-Code/lib)
target_link_libraries(SmartRobot_Rework_Uno PRIVATE
	MinimalSocket
)