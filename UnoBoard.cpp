#include "SmartRobot.h"
#include "UnoBoard.hpp"

namespace UnoBoard
{
    SmartRobot robot;
    void setup()
    {
        // put your setup code here, to run once:
        robot.SmartRobotInit();
    }

    void loop()
    {
        // put your main code here, to run repeatedly:
        if (!robot.isMoving)
            robot.getSerialData();
        else
        {
            robot.updateDistanceData();
        }
    }
}