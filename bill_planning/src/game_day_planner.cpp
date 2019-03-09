//#include "bill_planning/state_machine.hpp"
#include "bill_planning/sensor_readings.hpp"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"

StateMachine state_machine;

int current_heading = 0;
const float FULL_COURSE_SCAN_DISTANCE = 1.70;
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

    if (state_machine.major_state == INIT_SEARCH && state_machine.minor_state == RUN)
    {
        if(msg->data < FULL_COURSE_SCAN_DISTANCE)
        {
            state_machine.stateAction();
        }
    }
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    //SensorReadings::start_robot_performance_thread = true
}

void leftUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg) //left side maybe
{
    if (msg->data <= FULL_COURSE_SCAN_DISTANCE)
    {
        state_machine.stateAction();
    }
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    //SensorReadings::start_robot_performance_thread = true
}

void rightUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg) //left side maybe
{
    if (msg->data <= FULL_COURSE_SCAN_DISTANCE)
    {
        state_machine.stateAction();
    }
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    //SensorReadings::start_robot_performance_thread = true
}

void fireCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // If we run into fire while doing an initial search or searching for it
    if ((state_machine.major_state == SEARCH_FIRE || state_machine.major_state == INIT_SEARCH) &&
        state_machine.minor_state != FOUND_FIRE && msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        if (found_fire < 3)
        {
            found_fire++;
        }
        else if (state_machine.minor_state == RUN || state_machine.minor_state == VERIFY_FIRE)
        {
            state_machine.setFireFlag(true);
            state_machine.advanceState();
            found_fire = 0;  // Reset counter
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
    ros::Publisher state_pub;  //= nh.advertise<bill_msgs::State>("state", 100);
    ros::Publisher led_pub = nh.advertise<std_msgs::Bool>("led", 100);

    // Start state machine, then spin
    state_machine.setupPublishers(motor_pub, fan_pub, state_pub, led_pub);
    state_machine.start();

    ros::spin();
    return 0;
}
