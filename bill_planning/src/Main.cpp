#include "bill_planning/StateMachine.hpp"

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
void fireCallback(const std_msgs::Bool::ConstPtr& msg);
void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg);
void sideUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg);

ros::Publisher motor_pub;
ros::Publisher fan_pub;

StateMachine state_machine;

int current_heading = 0;
const int ULTRA_TRIGGER_DIST = 30;
int found_fire = 0;

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // This callback will in the future determine if a goal has been completed, but since the IMU
    // And the encoders aren't operational, it will have to just use timers instead
}

void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // This callback will determine what we should do with ultrasonic data
    // Logic needs to determine what we are searching for or if we need to mark a location
    // Logic needs to determine if we are close enough to complete a given task
    if (state_machine.search_state == MachineStates::INITIALSEARCH)
    {
        //We are doing the initial search 
    }
}

void sideUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // This callback will determine what we should do with ultrasonic data
    // Logic needs to determine what we are searching for or if we need to mark a location
    // Logic needs to determine if we are close enough to complete a given task
    if (state_machine.search_state == MachineStates::INITIALSEARCH)
    {
        //We are doing the initial search 
    }
}

void fireCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Only turn fan on if looking/scanning for fire and found fire
    if ((state_machine.search_state == MachineStates::LOOKFORFIRE
        || state_machine.search_state == MachineStates::SCANNINGFIRE2)
        && msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        if (found_fire < 3)
        {
            found_fire++;
        }
        else if (state_machine.search_state == MachineStates::LOOKFORFIRE)
        {
            state_machine.advanceState(current_heading);            
        }
        else
        {
            state_machine.resetScanningFire(current_heading);
        }
    }
    else
    {
        found_fire = 0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "game_day_planner");
    ros::NodeHandle nh;

    // Subscribing to Topics
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 1, fusedOdometryCallback);
    ros::Subscriber sub_fire = nh.subscribe("fire", 1, fireCallback);
    ros::Subscriber sub_ultrasonic = nh.subscribe("ultrasonic", 1, ultrasonicCallback);
    
    motor_pub = nh.advertise<bill_msgs::MotorCommands>("motor_cmd", 100);
    fan_pub = nh.advertise<std_msgs::Bool>("fan", 100);

    // Start state machine, then spin
    state_machine.planner.setPubs(motor_pub, fan_pub);
    state_machine.search_state = MachineStates::STARTINGCOURSE;
    state_machine.advanceState(current_heading);

    ros::spin();
    return 0;
}
