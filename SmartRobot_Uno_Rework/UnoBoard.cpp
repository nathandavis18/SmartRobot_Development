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
        robot.handle_incoming_data();
        if (robot.isMoving)
        {
            robot.updateDistanceData();
        }
    }
}