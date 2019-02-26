#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "bill_msgs/MotorCommands.h"
#include "bill_planning/include/bill_planning/Enums.hpp"
#include "bill_planning/include/bill_planning/State_Machine.hpp"
#include "bill_planning/include/bill_planning/Planner.hpp"

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
void fireCallback(const std_msgs::Bool::ConstPtr& msg);
void ultrasonicCallback(const std_msgs::Float32::ConstPtr& msg);

ros::Publisher motor_pub;
ros::Publisher fan_pub;
bill_msgs::MotorCommands command_msg;
State_Machine stateMachine;
Planner planner;

int StateMachine::state = 0;
int current_heading = 0;
const int ULTRA_TRIGGER_DIST = 30;

int foundFire = 0;

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // This callback will in the future determine if a goal has been completed, but since the IMU
    // And the encoders aren't operational, it will have to just use timers instead
}

void ultrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // This callback will determine what we should do with ultrasonic data
    // Logic needs to determine what we are searching for or if we need to mark a location
    // Logic needs to determine if we are close enough to complete a given task
}

void fireCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Only turn fan on if looking/scanning for fire and found fire
    if ((stateMachine::state == MachineStates::LOOKFORFIRE
        || stateMachine::state == MachineStates::SCANNINGFIRE
        || stateMachine::state == MachineStates::AWAI)
        && msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        // TODO: This may not work for scanning depending how fast we scan... might have
        //       only detect once while doing a scan
        if (foundFire < 3)
        {
            foundFire++;
        }
        else
        {
            planner.PublishStop();

            std_msgs::Bool cmd;
            cmd.data = true;

            fan_pub.publish(cmd); // Turn on fan
            ros::Duration(2).sleep();

            cmd.data = false;
            fan_pub.publish(cmd); // Turn off fan

            if (stateMachine::state == MachineStates::LOOKFORFIRE)
            {
                stateMachine.AdvanceState();
            }
            else
            {
                stateMachine.ContinueAngularScan();
            }
            
        }
    }
    else
    {
        foundFire = 0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_planner");
    ros::NodeHandle nh;

    planner = Planner();
    stateMachine.planner = planner;

    // Subscribing to Topics
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 1, fusedOdometryCallback);
    ros::Subscriber sub_fire = nh.subscribe("fire", 1, fireCallback);
    ros::Subscriber sub_ultrasonic = nh.subscribe("ultrasonic", 1, ultrasonicCallback);
    
    motor_pub = nh.advertise<bill_msgs::MotorCommands>("motor_cmd", 100);
    fan_pub = nh.advertise<std_msgs::Bool>("fan", 100);

    // Start state machine, then spin
    State_Machine.search_state = Enums::MachineStates::STARTINGCOURSE;
    State_Machine.AdvanceState();

    ros::spin();
    return 0;
}
