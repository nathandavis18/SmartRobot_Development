cmake_minimum_required(VERSION 3.14)
project(SmartRobot_Rework)

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

# --- Your executable ---
add_executable(SmartRobot_Rework
    "SmartRobot_Rework.cpp"
    "Smart-Robot-Code/SmartRobot_ESP32Board/MessageSerializer.cpp"
)

target_include_directories(SmartRobot_Rework PRIVATE lib Smart-Robot-Code/SmartRobot_ESP32Board Smart-Robot-Code/lib)
target_link_libraries(SmartRobot_Rework PRIVATE
    MinimalSocket
)
if(WIN32)
    target_link_libraries(SmartRobot_Rework PRIVATE ws2_32 wsock32)
endif()