#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <mutex>

std::mutex mutex;
geometry_msgs::Twist* reference = NULL;

void writeMotorRight(const unsigned int cmd)
{
    // TODO: Write a cmd to the right motor
}

void writeMotorLeft(const unsigned int cmd)
{
    // TODO: Write a cmd to the left motor
}

void pid()
{
    std::lock_guard<std::mutex> lk(mutex);
    unsigned int motor_cmd_right = 0;
    unsigned int motor_cmd_left = 0;
    // TODO: Determine input type to this function (speed/heading)
    // TODO: Use reference and input and compute motor commands

    writeMotorRight(motor_cmd_right);
    writeMotorLeft(motor_cmd_left);
}

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // TODO: Pull out heading, (speed?) as PID input
    // TODO: Call PID function
}

void motorCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lk(mutex);
    reference->angular = msg->angular;
    reference->linear = msg->linear;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver");
    ros::NodeHandle nh;
    ros::Subscriber sub_motor = nh.subscribe("motor_cmd", 1, motorCallback);
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 1, fusedOdometryCallback);

    ros::spin();
    return 0;
}
