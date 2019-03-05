#include "bill_planning/planner.hpp"

void Planner::setPubs(const ros::Publisher mp, const ros::Publisher fp, const ros::Publisher lp)
{
    _motor_pub = mp;
    _fan_pub = fp;
    _led_pub = lp;
}

void Planner::gridSearch()
{
    // TODO: Implement grid search
}

void Planner::publishStop()
{
    ROS_INFO("Commanding stop");
    _command_msg.command = bill_msgs::MotorCommands::STOP;
    _command_msg.heading = 0;
    _command_msg.speed = 0;
    _motor_pub.publish(_command_msg);
}

void Planner::publishDrive(const int heading, const float speed)
{
    ROS_INFO("Commanding drive, heading: %i and speed: %f", heading, speed);
    _command_msg.command = bill_msgs::MotorCommands::DRIVE;
    _command_msg.heading = heading;
    _command_msg.speed = speed;
    _motor_pub.publish(_command_msg);
}

void Planner::publishTurn(const int heading)
{
    ROS_INFO("Commanding turn, heading: %i", heading);
    _command_msg.command = bill_msgs::MotorCommands::TURN;
    _command_msg.heading = heading;
    _command_msg.speed = 0;  // Speed is hardcoded in the motor driver for turning
    _motor_pub.publish(_command_msg);
}

void Planner::putOutFire()
{
    publishStop();

    // Turn ON/OFF Fan
    std_msgs::Bool cmd;
    cmd.data = true;
    _fan_pub.publish(cmd);  // Turn on fan
    ros::Duration(2).sleep();
    cmd.data = false;
    _fan_pub.publish(cmd);  // Turn off fan
}

void Planner::signalComplete()
{
    publishStop();
    std_msgs::Bool msg;
    msg.data = true;
    _led_pub.publish(msg);
    ros::Duration(2).sleep();
    msg.data = false;
    _led_pub.publish(msg);
}