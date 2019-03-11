#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_planning/sensor_readings.hpp"
#include <math.h>
#include "bill_planning/position.hpp"
#include <cstddef>


int current_heading = 0;
int found_fire = 0;
unsigned char start_course = 0x00;

Planner planner;

//IF YOU DON'T DECLARE STATIC MEMBERS WITH A VALUE, IT WONT BUILD
TilePosition SensorReadings::current_tile(0,0);
int SensorReadings::current_heading = 90;
bool SensorReadings::detected_fire = false;
bool SensorReadings::start_robot_performance_thread = false;
float SensorReadings::ultra_fwd = -500;
float SensorReadings::ultra_left = -500;
float SensorReadings::ultra_right = -500;
unsigned char SensorReadings::detection_bit = 0x00;
Planner SensorReadings::planner = planner;
TilePosition SensorReadings::currentTargetPoint(0,0);
STATE SensorReadings::current_state = STATE::INIT_SEARCH;


void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_heading = (int)angles::to_degrees(tf::getYaw(msg->pose.pose.orientation));
    // UPDATE HEADING AND POSSIBLY POSITION IN SENSOR READINGS

    // Truncate this instead of floor to
    int currentX = (int)std::trunc(msg->pose.pose.position.x);
    int currentY = (int)std::trunc(msg->pose.pose.position.y);

    SensorReadings::current_tile.x = currentX;
    SensorReadings::current_tile.y = currentY;

    // This may cause weird behaviour when the robot is on the edges of a tile
    if (SensorReadings::currentTargetPoint.x == currentX && SensorReadings::currentTargetPoint.y == currentY)
    {
        // We have arrived at our current target point
        planner.ProcessNextDrivePoint(currentX, currentY);
    }
}

void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(msg->data > 400)
    {
        // NOT POSSIBLE IN THE COURSE ~ BAD DATA
        return;
    }

    // TODO DON'T USE 170
    // HOW CAN WE DISTINGUISH BETW WALL AND OBJECT HERE? 
    // If we have increased then decreased over past 5 msrmts?
    if (SensorReadings::current_state == STATE::INIT_FIRE_SCAN && msg->data < 170)
    {
        // Mark New Object Using Math STUFF
    }
    
    SensorReadings::ultra_fwd = msg->data;
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x01;
    if(!SensorReadings::start_robot_performance_thread 
        && 0x07 - start_course == 0x00)
    {
        SensorReadings::start_robot_performance_thread = true;
    }
}

void leftUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg) //left side maybe
{
    if(msg->data > 400)
    {
        // NOT POSSIBLE IN THE COURSE ~ BAD DATA
        return;
    }

    if (SensorReadings::current_state == STATE::INIT_SEARCH && msg->data < 170)
    {
        // Mark New Object Using Math
    }

    SensorReadings::ultra_left = msg->data;
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x02;
    if(!SensorReadings::start_robot_performance_thread 
        && 0x07 - start_course == 0x00)
    {
        SensorReadings::start_robot_performance_thread = true;
    }
}

void rightUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg) //left side maybe
{
    if(msg->data > 400)
    {
        // NOT POSSIBLE IN THE COURSE ~ BAD DATA
        return;
    }
    
    if (SensorReadings::current_state == STATE::INIT_SEARCH && msg->data < 170)
    {
        // Mark New Object Using Math
    }

    SensorReadings::ultra_right = msg->data;
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x04;
    if(!SensorReadings::start_robot_performance_thread 
        && 0x07 - start_course == 0x00)
    {
        SensorReadings::start_robot_performance_thread = true;
    }
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

    planner.setPubs(motor_pub, fan_pub, led_pub);
    SensorReadings::planner = planner;

    ros::spin();
    return 0;
}
