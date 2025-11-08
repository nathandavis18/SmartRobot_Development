#include "SmartRobot.h"
#include "UnoBoard.hpp"

namespace UnoBoard
{
    static sr::SmartRobot robot;
    void setup()
    {
        robot.SmartRobotInit();
    }

    void loop()
    {
        if (!robot.isMoving)
            robot.handle_incoming_data();
        else
        {
            robot.updateDistanceData();
        }
    }
}