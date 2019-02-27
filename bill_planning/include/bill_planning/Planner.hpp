#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "bill_msgs/MotorCommands.h"

class Planner
{
    public:
    	void setMotorPub(ros::Publisher mp);        
        void gridSearch();
        void scanAngle(int heading);
        void publishStop();
        void publishDrive(const int heading, const float speed);
        void publishTurn(const int heading);
    private:
        bill_msgs::MotorCommands command_msg;
        ros::Publisher motor_pub;
};
