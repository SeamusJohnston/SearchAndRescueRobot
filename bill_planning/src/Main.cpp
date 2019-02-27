#include "bill_planning/StateMachine.hpp"

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
void fireCallback(const std_msgs::Bool::ConstPtr& msg);
void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg);
void sideUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg);

ros::Publisher motor_pub;
ros::Publisher fan_pub;

StateMachine state_machine;

int current_heading = 0;
const float ULTRA_INIT_SCAN_DIST = 163.83;
const float ULTRA_CLOSETOWALL_DIST = 5;
int found_fire = 0;

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // This callback will in the future determine if a goal has been completed, but since the IMU
    // And the encoders aren't operational, it will have to just use timers instead
}

void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Logic needs to determine if we are close enough to complete a given task
    if (state_machine.search_state == MachineStates::INITIALSEARCH
        && msg->data <= ULTRA_INIT_SCAN_DIST)
    {
        // Either save point and move around object ||
        // End of path, how far have we travelled? Global variable may help here
    }

    // TODO: Turn when hit a wall
    // TODO: Read sensors to determine what we found
    // TODO: Mark object with type or signal object completion
}

void sideUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    //Scan up to the end of course - width building - width robot turning radius
    // TODO: How do we move around objects directly in our path
    if (state_machine.search_state == MachineStates::INITIALSEARCH
        && msg->data <= ULTRA_CLOSETOWALL_DIST)
    {
        // We are done the initial search
        state_machine.advanceState();
    }
    // TODO: If we are completing our search and ever find something worth noting
    //          we should mark point or signal object completion depending on
    //          sensor reading (Must turn to face the object of interest)
    //          what is a good distance for regular search? 10-20 cm might suffice
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
    state_machine.search_state = MachineStates::STARTINGCOURSE; //State 0
    state_machine.advanceState(current_heading);

    ros::spin();
    return 0;
}
