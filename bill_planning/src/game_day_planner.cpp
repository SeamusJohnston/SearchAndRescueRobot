#include "bill_planning/state_machine.hpp"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"

StateMachine state_machine;

int current_heading = 0;
const float ULTRA_INIT_SCAN_DIST = 163.83;
const float ULTRA_CLOSETOWALL_DIST = 5;
int found_fire = 0;

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // This callback will in the future determine if a goal has been completed
    current_heading = (int)angles::to_degrees(tf::getYaw(msg->pose.pose.orientation));
    state_machine.updateHeading(current_heading);
}

void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Logic needs to determine if we are close enough to complete a given task
    // TODO: Bryan implement
    if (state_machine.major_state == INIT_SEARCH
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
    // TODO: Bryan implement
    if (state_machine.major_state == INIT_SEARCH
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
    // If we see and fire, and we are searching for one, but not if we already found it (don't log if we are currently extinguishing)
    if (state_machine.major_state == SEARCH_FIRE && state_machine.minor_state != FOUND_FIRE && msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        if (found_fire < 3)
        {
            found_fire++;
        }
        else if (state_machine.minor_state == RUN)
        {
            state_machine.setFireFlag(true);
            state_machine.advanceState();
            found_fire = 0; // Reset counter
        }
        else if (state_machine.minor_state == VERIFY_FIRE)
        {
            // If we see a fire while verifying, inform the state machine
            state_machine.setFireFlag(true);
            state_machine.advanceState();
            found_fire = 0; // Reset counter
        }
    }
    else
    {
        found_fire = 0;
    }
}

void resetCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Reset all flags, and reset the state machine
    found_fire = 0;
    state_machine.reset();
    state_machine.start();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "game_day_planner");
    ros::NodeHandle nh;

    // Subscribing to Topics
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 1, fusedOdometryCallback);
    ros::Subscriber sub_fire = nh.subscribe("fire", 1, fireCallback);
    ros::Subscriber sub_ultrasonic = nh.subscribe("ultrasonic", 1, frontUltrasonicCallback);
    ros::Subscriber sub_reset = nh.subscribe("reset", 1, resetCallback);
    
    ros::Publisher motor_pub = nh.advertise<bill_msgs::MotorCommands>("motor_cmd", 100);
    ros::Publisher fan_pub = nh.advertise<std_msgs::Bool>("fan", 100);
    ros::Publisher state_pub; //= nh.advertise<bill_msgs::State>("state", 100);
    ros::Publisher led_pub = nh.advertise<std_msgs::Bool>("led", 100);

    // Start state machine, then spin
    state_machine.setupPublishers(motor_pub, fan_pub, state_pub, led_pub);
    state_machine.start();

    ros::spin();
    return 0;
}
