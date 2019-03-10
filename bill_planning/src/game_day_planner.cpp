#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_planning/sensor_readings.hpp"
#include <math.h>


int current_heading = 0;
int found_fire = 0;

Planner planner;
//SensorReadings sensor_readings;

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_heading = (int)angles::to_degrees(tf::getYaw(msg->pose.pose.orientation));
    // UPDATE HEADING AND POSSIBLY POSITION IN SENSOR READINGS

    // Truncate this instead of floor to
    int currentX = (int)std::trunc(msg->pose.pose.position.x);
    int currentY = (int)std::trunc(msg->pose.pose.position.y);

    // This may cause weird behaviour when the robot is on the edges of a tile
    if (SensorReadings::currentTargetPoint.x == currentX && SensorReadings::currentTargetPoint.y == currentY)
    {
        // We have arrived at our current target point
        planner.ProcessNextDrivePoint(SensorReadings::currentTargetPoint.x, SensorReadings::currentTargetPoint.y);
    }
}

void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    //UPDATE FRONT ULTRA IN SENSOR READINGS
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    //SensorReadings::start_robot_performance_thread = true
}

void leftUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg) //left side maybe
{
    //UPDATE LEFT ULTRA IN SENSOR READINGS
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    //SensorReadings::start_robot_performance_thread = true
}

void rightUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg) //left side maybe
{
    //UPDATE RIGHT ULTRA IN SENSOR READINGS
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    //SensorReadings::start_robot_performance_thread = true
}

void fireCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        if (found_fire < 3)
        {
            found_fire++;
            //sensor_readings.detected_fire = false;
        }
        else
        {
            //sensor_readings.detected_fire = true;
        }
    }
    else
    {
        found_fire = 0;
        //sensor_readings.detected_fire = false;
    }
}

void resetCallback(const std_msgs::Bool::ConstPtr& msg)
{
    //WRITE FUNCTION TO RESET SENSOR READINGS
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

    planner.setPubs(motor_pub, fan_pub, led_pub);
    //sensor_readings.setPlanner(planner);
    ros::spin();
    return 0;
}
