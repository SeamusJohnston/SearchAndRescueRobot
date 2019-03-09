#include "bill_planning/sensor_readings.hpp"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"

int current_heading = 0;
const float FULL_COURSE_SCAN_DISTANCE = 1.70;
const float ULTRA_CLOSETOWALL_DIST = 5;
int found_fire = 0;

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_heading = (int)angles::to_degrees(tf::getYaw(msg->pose.pose.orientation));
    // UPDATE HEADING AND POSSIBLY POSITION IN SENSOR READINGS
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
            SensorReadings::detected_fire = false;
        }
        else
        {
            SensorReadings::detected_fire = true;
        }
    }
    else
    {
        found_fire = 0;
        SensorReadings::detected_fire = false;
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

    ros::spin();
    return 0;
}
