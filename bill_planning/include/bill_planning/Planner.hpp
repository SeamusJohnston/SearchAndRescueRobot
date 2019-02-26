#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "bill_msgs/MotorCommands.h"

#define ULTRA_DIST 30
class Planner
{
    public:
        Planner();        
        void GridSearch();
        void ScanAngle(int heading);
        void PublishStop();
        void PublishDrive(const int heading, const float speed);
        void PublishTurn(const int heading);
}
