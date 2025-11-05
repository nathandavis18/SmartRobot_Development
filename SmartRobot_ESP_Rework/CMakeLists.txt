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

project(SmartRobot_Rework_ESP)

add_executable(SmartRobot_Rework_ESP
	"EspMain.cpp"
    "SmartRobot_Rework_ESP.cpp"
    "ESP32Board.cpp" "ESP32Board.hpp"
    "../Smart-Robot-Code/SmartRobot_ESP32Board/MessageSerializer.cpp"
	"../lib/Arduino.h")

target_include_directories(SmartRobot_Rework_ESP PRIVATE ../lib ../Smart-Robot-Code/SmartRobot_ESP32Board ../Smart-Robot-Code/lib)
target_link_libraries(SmartRobot_Rework_ESP PRIVATE
    MinimalSocket
)