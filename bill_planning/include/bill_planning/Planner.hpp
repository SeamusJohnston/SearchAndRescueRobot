#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "bill_msgs/MotorCommands.h"

class Planner
{
    public:
        Planner();
	void setMotorPub(ros::Publisher mp);        
        void GridSearch();
        void ScanAngle(int heading);
        void PublishStop();
        void PublishDrive(const int heading, const float speed);
        void PublishTurn(const int heading);
    private:
        bill_msgs::MotorCommands command_msg;
        ros::Publisher motor_pub;
};
