#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_planning/sensor_readings.hpp"
#include <math.h>
#include "bill_planning/position.hpp"

#include <cstddef>
#include <iostream>
#include <utility>
#include <thread>
#include <chrono>

int current_heading = 0;
int found_fire = 0;
unsigned char start_course = 0x00;

Planner planner;

// FUNCTIONS
void fireOut();
void findClearPathFwd();
void completeStraightLineSearch();
void scanForFire();
void driveToDesiredPoints();
//TODO
void conductGridSearch();
void driveToLargeBuilding();
void secondMainFunction(int n);

//IF YOU DON'T DECLARE STATIC MEMBERS WITH A VALUE, IT WONT BUILD
TilePosition SensorReadings::current_tile(0,0);
int SensorReadings::current_heading = 90;
bool SensorReadings::detected_fire = false;
bool SensorReadings::start_robot_performance_thread = false;
float SensorReadings::ultra_fwd = -500;
float SensorReadings::ultra_left = -500;
float SensorReadings::ultra_right = -500;
unsigned char SensorReadings::detection_bit = 0x00;
Planner * SensorReadings::planner = planner;
TilePosition SensorReadings::currentTargetPoint(0,0);
STATE SensorReadings::current_state = STATE::INIT_SEARCH;
std::queue<TilePosition> SensorReadings::points_of_interest;


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
    ROS_INFO("Front Ultra Callback: %f \n", msg->data);
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
    ROS_INFO("Flame Callback: %i \n", SensorReadings::detected_fire);
    SensorReadings::detected_fire = msg->data;
    ROS_INFO("Flame Callback: %i \n", msg->data);
    ROS_INFO("Flame Callback: %i \n", SensorReadings::detected_fire);

    if (msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        if (found_fire < 3)
        {
            found_fire++;
            SensorReadings::detected_fire = true;
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

// TODO MAKE CALLBACK FOR HALL EFFECT
// STORE POINT IF IT EVER GOES OFF
// DEFAULT TO -100,-100

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

    //ROS_INFO("MEMORY ADDRESS: %x", &planner);
    SensorReadings::planner = &planner;
    //ROS_INFO("MEMORY ADDRESS: %x", SensorReadings::planner);

    std::thread robot_execution_thread(secondMainFunction, 5);
    ros::spin();
    robot_execution_thread.join();
    return 0;
}

// FLAGS
bool _cleared_fwd = false;
bool _driven_fwd = false;

// POSITION
TilePosition desired_tile(0,0);
TilePosition large_building_tile(0,0);
int desired_heading = 90;

// CONSTANTS
const int FULL_COURSE_DETECTION_LENGTH = 1.70;
const int FIRE_SCAN_ANGLE = 20;

void secondMainFunction(int n)
{
    ROS_INFO("RUNNING SEPARATE THREAD: %i", n);
    // WAIT ON DATA FROM EACH ULTRASONIC SENSOR
    while (!SensorReadings::start_robot_performance_thread)
    {
        ROS_INFO("FIRE READING: %i", SensorReadings::detected_fire);
        ROS_INFO("FIRE READING: %f", SensorReadings::ultra_fwd);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    ROS_INFO("PASSED FIRST WHILE LOOP");
    findClearPathFwd();

    completeStraightLineSearch();
    SensorReadings::current_state = STATE::INIT_FIRE_SCAN;

    scanForFire();

    if(SensorReadings::points_of_interest.size() < 3)
    {
        // SOMETHING WENT WRONG WE SHOULD HAVE DETECTED BY NOW
        // THIS PROBABLY MEANS THEY ARE BACK TO BACK
    }

    SensorReadings::current_state = STATE::FLAME_SEARCH;
    driveToDesiredPoints();
}

void fireOut()
{
    bool initialCall = true;
    bool check_temp_heading = true;

    int temp_desired_heading = 0;

    do
    {
        if (initialCall || SensorReadings::detected_fire)
        {
            SensorReadings::planner->putOutFire();
            desired_heading = SensorReadings::current_heading + 2 * FIRE_SCAN_ANGLE;
            temp_desired_heading = SensorReadings::current_heading - FIRE_SCAN_ANGLE;

            SensorReadings::planner->publishTurn(temp_desired_heading);

            initialCall = false;
            check_temp_heading = true;
        }

        while(check_temp_heading
              && temp_desired_heading != SensorReadings::current_heading)
        {
            if(SensorReadings::detected_fire)
            {
                SensorReadings::planner->putOutFire();
                desired_heading = SensorReadings::current_heading + 2 * FIRE_SCAN_ANGLE;
                temp_desired_heading = SensorReadings::current_heading - FIRE_SCAN_ANGLE;
                SensorReadings::planner->publishTurn(temp_desired_heading);
            }
        }

        check_temp_heading = false;
        SensorReadings::planner->publishTurn(desired_heading);
    } while(desired_heading != SensorReadings::current_heading);

    SensorReadings::current_state = STATE::BUILDING_SEARCH;
}

void findClearPathFwd()
{
    if(!_cleared_fwd && SensorReadings::ultra_fwd >= FULL_COURSE_DETECTION_LENGTH)
    {
        _cleared_fwd = true;
    }
    else
    {
        int increment = SensorReadings::ultra_left > SensorReadings::ultra_right ? -1 : 1;
        desired_heading = SensorReadings::ultra_left > SensorReadings::ultra_right ? 180 : 0;

        while(!_cleared_fwd)
        {
            int temp_ultra = increment == -1 ?
                             SensorReadings::ultra_right :
                             SensorReadings::ultra_left;

            if (desired_tile.x == SensorReadings::current_tile.x
                && desired_tile.y == SensorReadings::current_tile.y
                && temp_ultra < FULL_COURSE_DETECTION_LENGTH)
            {
                SensorReadings::planner->publishDriveToTile(
                        SensorReadings::current_tile.x,
                        SensorReadings::current_tile.y,
                        SensorReadings::current_tile.x + increment,
                        0, 0.4);
                // THE ULTRASONIC CALLBACK WILL BE IN CHARGE OF SAVING THE POINT OF INTEREST
                desired_tile.x = SensorReadings::current_tile.x + increment;
                desired_tile.y = 0;
            }
            else if (temp_ultra > FULL_COURSE_DETECTION_LENGTH)
            {
                _cleared_fwd = true;
            }
        }

        desired_heading = 90;
        SensorReadings::planner->publishTurn(desired_heading);
    }
}

void completeStraightLineSearch()
{
    desired_tile.x = SensorReadings::current_tile.x;
    desired_tile.y = 6;

    SensorReadings::planner->publishDriveToTile(SensorReadings::current_tile.x,
                                                SensorReadings::current_tile.y,
                                                desired_tile.x,
                                                desired_tile.y, 0.4);

    while(!_driven_fwd)
    {
        if(desired_tile.x == SensorReadings::current_tile.x
           && desired_tile.y == SensorReadings::current_tile.y)
        {
            _driven_fwd = true;
        }
    }
}

void scanForFire()
{
    desired_heading = 0;
    SensorReadings::planner->publishTurn(desired_heading);
    while (SensorReadings::current_heading != desired_heading);

    // 181 ENSURES FRONT WILL DO THE SCANNING (WE ARE AT TOP OF GRID)
    // AND THE ULTRASONIC CAN AID IN OBJECT DETECTION
    desired_heading = 181;
    SensorReadings::planner->publishTurn(desired_heading);
    while (SensorReadings::current_heading != desired_heading);
}

void driveToDesiredPoints()
{
    while (!SensorReadings::points_of_interest.empty())
    {
        SensorReadings::currentTargetPoint = SensorReadings::points_of_interest.front();
        SensorReadings::points_of_interest.pop();

        desired_tile.x = SensorReadings::currentTargetPoint.x;
        desired_tile.y = SensorReadings::currentTargetPoint.y;

        SensorReadings::planner->publishDriveToTile(SensorReadings::current_tile.x,
                                                    SensorReadings::current_tile.y,
                                                    desired_tile.x,
                                                    desired_tile.y, 0.4);

        //Drive to position
        while (SensorReadings::current_tile.x != desired_tile.x
               && SensorReadings::current_tile.y != desired_tile.y);

        // TODO BEFORE WHILE
        // We could possibly trigger a scan in here but maybe we should create a scan tile
        // function in planner to search tile for one of the 3 objects
        while (SensorReadings::detection_bit == 0x00)
        {
        }

        if (SensorReadings::detection_bit == 0x01)
        {
            fireOut();
        }
        else if (SensorReadings::current_state = STATE::BUILDING_SEARCH)
        {
            SensorReadings::planner->signalComplete();
            if (SensorReadings::detection_bit == 0x03)
            {
                large_building_tile.x = SensorReadings::current_tile.x;
                large_building_tile.x = SensorReadings::current_tile.y;
            }
        }
        else
        {
            SensorReadings::points_of_interest.push(SensorReadings::currentTargetPoint);
        }
    }
}


void conductGridSearch()
{

}

void driveToLargeBuilding()
{
    SensorReadings::currentTargetPoint.x = large_building_tile.x;
    SensorReadings::currentTargetPoint.y = large_building_tile.y;

    desired_tile.x = large_building_tile.x;
    desired_tile.y = large_building_tile.y;


    SensorReadings::planner->publishDriveToTile(SensorReadings::current_tile.x,
                                                SensorReadings::current_tile.y,
                                                desired_tile.x,
                                                desired_tile.y, 0.4);

    while (SensorReadings::current_tile.x != desired_tile.x
           && SensorReadings::current_tile.y != desired_tile.y);

    // TODO
    // We could possibly trigger a scan in here but maybe we should create a scan tile
    // function in planner to search tile for one of the 3 objects
    while (SensorReadings::detection_bit == 0x00)
    {
    }
}