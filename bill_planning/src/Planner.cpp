#include "bill_planner/Planner.hpp"

Planner::Planner(State_Machine sm)
{
}

Planner::GridSearch()
{
}

Planner::ScanAngle(int heading)
{
    publishTurn(heading % 360);
}

Planner::PublishStop()
{
    //std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding stop");
    command_msg.command = bill_msgs::MotorCommands::STOP;
    command_msg.heading = 0;
    command_msg.speed = 0;
    motor_pub.publish(command_msg);
}

Planner::PublishDrive(const int heading, const float speed)
{
    //std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding drive, heading: %i and speed: %f", heading, speed);
    command_msg.command = bill_msgs::MotorCommands::DRIVE;
    command_msg.heading = heading;
    command_msg.speed = speed;
    motor_pub.publish(command_msg);
}

Planner::PublishTurn(const int heading)
{
    //std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding turn, heading: %i", heading);
    command_msg.command = bill_msgs::MotorCommands::TURN;
    command_msg.heading = heading;
    command_msg.speed = 0; // Speed is hardcoded in the motor driver for turning
    motor_pub.publish(command_msg);
}