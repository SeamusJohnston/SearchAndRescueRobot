#include "bill_planning/Planner.hpp"


void Planner::setMotorPub(ros::Publisher mp)
{
    motor_pub = mp;
}

void Planner::gridSearch()
{
}

void Planner::publishStop()
{
    //std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding stop");
    command_msg.command = bill_msgs::MotorCommands::STOP;
    command_msg.heading = 0;
    command_msg.speed = 0;
    motor_pub.publish(command_msg);
}

void Planner::publishDrive(const int heading, const float speed)
{
    //std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding drive, heading: %i and speed: %f", heading, speed);
    command_msg.command = bill_msgs::MotorCommands::DRIVE;
    command_msg.heading = heading;
    command_msg.speed = speed;
    motor_pub.publish(command_msg);
}

void Planner::publishTurn(const int heading)
{
    //std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding turn, heading: %i", heading);
    command_msg.command = bill_msgs::MotorCommands::TURN;
    command_msg.heading = heading;
    command_msg.speed = 0; // Speed is hardcoded in the motor driver for turning
    motor_pub.publish(command_msg);
}
