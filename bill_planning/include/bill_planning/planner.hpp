#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "bill_msgs/MotorCommands.h"

class Planner
{
  public:
    void setPubs(ros::Publisher mp, ros::Publisher fp, ros::Publisher lp);
    void gridSearch();
    void publishStop();
    void publishDrive(int heading, float speed);
    void publishTurn(int heading);
    void putOutFire();
    void signalComplete();

  private:
    bill_msgs::MotorCommands _command_msg;
    ros::Publisher _motor_pub;
    ros::Publisher _fan_pub;
    ros::Publisher _led_pub;
};

#endif