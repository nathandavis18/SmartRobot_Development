#include "SmartRobot.h"
#include "UnoBoard.hpp"

namespace UnoBoard
{
    sr::SmartRobot robot;
    void setup()
    {
        robot.SmartRobotInit();
    }

    void loop()
    {
        if (!robot.isMoving)
            robot.getSerialData();
        else
        {
            robot.updateDistanceData();
        }
    }
}