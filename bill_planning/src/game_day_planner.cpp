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

// PLAY WITH THREAD SLEEP IN robotPerformanceThread

// FUNCTIONS
void fireOut();
void findClearPathFwd();
void completeStraightLineSearch();
void scanForFire();
void driveToDesiredPoints();
void robotPerformanceThread(int n);
//TODO IMPLEMENT
void conductGridSearch();
void driveToLargeBuilding();

SensorReadings sensor_readings();

int found_fire = 0;
unsigned char start_course = 0x00;
Planner planner;

// FLAGS
bool _cleared_fwd = false;
bool _driven_fwd = false;
bool KILL_SWITCH = false; 

// POSITION
TilePosition desired_tile(0,0);
TilePosition large_building_tile(0,0);
int desired_heading = 90;

// CONSTANTS
const int FULL_COURSE_DETECTION_LENGTH = 1.70;
const int FIRE_SCAN_ANGLE = 20;

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    sensor_readings.setCurrentHeading((int)angles::to_degrees(tf::getYaw(msg->pose.pose.orientation)));
    // UPDATE HEADING AND POSSIBLY POSITION IN SENSOR READINGS

    // Truncate this instead of floor to
    int currentX = (int)std::trunc(msg->pose.pose.position.x);
    int currentY = (int)std::trunc(msg->pose.pose.position.y);

    // This may cause weird behaviour when the robot is on the edges of a tile
    if (sensor_readings.currentTargetPoint.x == currentX && sensor_readings.currentTargetPoint.y == currentY)
    {
        // We have arrived at our current target point
        planner.ProcessNextDrivePoint(sensor_readings.currentTargetPoint.x, sensor_readings.currentTargetPoint.y);
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
    if (sensor_readings.getCurrentState() == STATE::INIT_FIRE_SCAN && msg->data < 170)
    {
        // Mark New Object Using Math STUFF
    }

    sensor_readings.setUltraFwd(msg->data);
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x01;
    if(!sensor_readingsgetStartRobotPerformanceThread())
       && 0x07 - start_course == 0x00)
    {
        sensor_readingssetStartRobotPerformanceThread(true);
    }
}

void leftUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg) //left side maybe
{
    if(msg->data > 400)
    {
        // NOT POSSIBLE IN THE COURSE ~ BAD DATA
        return;
    }

    if (sensor_readings.getCurrentState() == STATE::INIT_SEARCH && msg->data < 170)
    {
        // Mark New Object Using Math
    }

    sensor_readings.setUltraLeft(msg->data);
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x02;
    if(!sensor_readingsgetStartRobotPerformanceThread())
       && 0x07 - start_course == 0x00)
    {
        sensor_readingssetStartRobotPerformanceThread(true);
    }
}

void rightUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg) //left side maybe
{
    if(msg->data > 400)
    {
        // NOT POSSIBLE IN THE COURSE ~ BAD DATA
        return;
    }

    if (sensor_readings.getCurrentState() == STATE::INIT_SEARCH && msg->data < 170)
    {
        // Mark New Object Using Math
    }

    sensor_readings.setUltraRight(msg->data);
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x04;
    if(!sensor_readingsgetStartRobotPerformanceThread())
       && 0x07 - start_course == 0x00)
    {
        sensor_readingssetStartRobotPerformanceThread(true);
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
            sensor_readings.setDetectedFire(false);
        }
        else
        {
            sensor_readings.setDetectedFire(true);
        }
    }
    else
    {
        found_fire = 0;
        sensor_readings.setDetectedFire(false);
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

    std::thread robot_execution_thread(robotPerformanceThread, 5);
    ros::spin();
    KILL_SWITCH = true;
    robot_execution_thread.join();
    return 0;
}

void robotPerformanceThread(int n)
{
    ROS_INFO("RUNNING SEPARATE THREAD: %i", n);
    // WAIT ON DATA FROM EACH ULTRASONIC SENSOR
    while (!sensor_readingsgetStartRobotPerformanceThread() && !KILL_SWITCH)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ROS_INFO("PASSED FIRST WHILE LOOP");
    findClearPathFwd();

    completeStraightLineSearch();
    sensor_readings.setCurrentState(STATE::INIT_FIRE_SCAN);

    scanForFire();

    if(sensor_readings.points_of_interest.size() < 3)
    {
        // SOMETHING WENT WRONG WE SHOULD HAVE DETECTED BY NOW
        // THIS PROBABLY MEANS THEY ARE BACK TO BACK
    }

    sensor_readings.setCurrentState(STATE::FLAME_SEARCH);
    driveToDesiredPoints();
}

void fireOut()
{
    bool initialCall = true;
    bool check_temp_heading = true;

    int temp_desired_heading = 0;

    do
    {
        if (initialCall || sensor_readings.getDetectedFire())
        {
            planner.putOutFire();
            desired_heading = sensor_readings.getCurrentHeading() + 2 * FIRE_SCAN_ANGLE;
            temp_desired_heading = sensor_readings.getCurrentHeading() - FIRE_SCAN_ANGLE;

            planner.publishTurn(temp_desired_heading);

            initialCall = false;
            check_temp_heading = true;
        }

        while(check_temp_heading
              && temp_desired_heading != sensor_readings.getCurrentHeading()
              && !KILL_SWITCH)
        {
            if(sensor_readings.getDetectedFire())
            {
                planner.putOutFire();
                desired_heading = sensor_readings.getCurrentHeading() + 2 * FIRE_SCAN_ANGLE;
                temp_desired_heading = sensor_readings.getCurrentHeading() - FIRE_SCAN_ANGLE;
                planner.publishTurn(temp_desired_heading);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        check_temp_heading = false;
        planner.publishTurn(desired_heading);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while(desired_heading != sensor_readings.getCurrentHeading() && !KILL_SWITCH);

    sensor_readings.setCurrentState(STATE::BUILDING_SEARCH);
}

void findClearPathFwd()
{
    if(!_cleared_fwd && sensor_readings.getUltraFwd() >= FULL_COURSE_DETECTION_LENGTH)
    {
        _cleared_fwd = true;
    }
    else
    {
        int increment = sensor_readings.getUltraLeft() > sensor_readings.getUltraRight() ?
            -1 : 1;
        desired_heading = sensor_readings.getUltraLeft() > sensor_readings.getUltraRight() ?
            180 : 0;

        while(!_cleared_fwd && !KILL_SWITCH)
        {
            int temp_ultra = increment == -1 ?
                             sensor_readings.getUltraRight() :
                             sensor_readings.getUltraLeft();

            if (desired_tile.x == sensor_readings.getCurrentTileX()
                && desired_tile.y == sensor_readings.getCurrentTileY()
                && temp_ultra < FULL_COURSE_DETECTION_LENGTH)
            {
                planner.publishDriveToTile(
                        sensor_readings.getCurrentTileX(),
                        sensor_readings.getCurrentTileY(),
                        sensor_readings.getCurrentTileX() + increment,
                        0, 0.4);
                // THE ULTRASONIC CALLBACK WILL BE IN CHARGE OF SAVING THE POINT OF INTEREST
                desired_tile.x = sensor_readings.getCurrentTileX() + increment;
                desired_tile.y = 0;
            }
            else if (temp_ultra > FULL_COURSE_DETECTION_LENGTH)
            {
                _cleared_fwd = true;
            }
        }

        desired_heading = 90;
        planner.publishTurn(desired_heading);
    }
}

void completeStraightLineSearch()
{
    desired_tile.x = sensor_readings.getCurrentTileX();
    desired_tile.y = 6;

    planner.publishDriveToTile(sensor_readings.getCurrentTileX(),
        sensor_readings.getCurrentTileY(),
        desired_tile.x,
        desired_tile.y, 0.4);

    while(!_driven_fwd && !KILL_SWITCH)
    {
        if(desired_tile.x == sensor_readings.getCurrentTileX()
           && desired_tile.y == sensor_readings.getCurrentTileY())
        {
            _driven_fwd = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void scanForFire()
{
    desired_heading = 0;
    planner.publishTurn(desired_heading);
    while (sensor_readings.getCurrentHeading() != desired_heading && !KILL_SWITCH)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 181 ENSURES FRONT WILL DO THE SCANNING (WE ARE AT TOP OF GRID)
    // AND THE ULTRASONIC CAN AID IN OBJECT DETECTION
    desired_heading = 181;
    planner.publishTurn(desired_heading);
    while (sensor_readings.getCurrentHeading() != desired_heading && !KILL_SWITCH)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void driveToDesiredPoints()
{
    while (!sensor_readings.points_of_interest.empty() && !KILL_SWITCH)
    {
        sensor_readings.currentTargetPoint = sensor_readings.points_of_interest.front();
        sensor_readings.points_of_interest.pop();

        desired_tile.x = sensor_readings.currentTargetPoint.x;
        desired_tile.y = sensor_readings.currentTargetPoint.y;

        planner.publishDriveToTile(sensor_readings.getCurrentTileX(),
            sensor_readings.getCurrentTileY(),
            desired_tile.x,
            desired_tile.y, 0.4);

        //Drive to position
        while ((sensor_readings.getCurrentTileX() != desired_tile.x
               || sensor_readings.getCurrentTileY() != desired_tile.y)
               && !KILL_SWITCH)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // TODO BEFORE WHILE
        // We could possibly trigger a scan in here but maybe we should create a scan tile
        // function in planner to search tile for one of the 3 objects
        while (sensor_readings.getDetectionBit() == 0x00
            && !KILL_SWITCH)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (sensor_readings.getDetectionBit() == 0x01)
        {
            fireOut();
        }
        else if (sensor_readings.getCurrentState() = STATE::BUILDING_SEARCH)
        {
            planner.signalComplete();
            if (sensor_readings.getDetectionBit() == 0x03)
            {
                large_building_tile.x = sensor_readings.getCurrentTileX();
                large_building_tile.x = sensor_readings.getCurrentTileY();
            }
        }
        else
        {
            sensor_readings.points_of_interest.push(sensor_readings.currentTargetPoint);
        }
    }
}

void conductGridSearch()
{

}

void driveToLargeBuilding()
{
    sensor_readings.currentTargetPoint.x = large_building_tile.x;
    sensor_readings.currentTargetPoint.y = large_building_tile.y;

    desired_tile.x = large_building_tile.x;
    desired_tile.y = large_building_tile.y;

    planner.publishDriveToTile(sensor_readings.getCurrentTileX(),
        sensor_readings.getCurrentTileY(),
        desired_tile.x,
        desired_tile.y, 0.4);

    while ((sensor_readings.getCurrentTileX() != desired_tile.x
           || sensor_readings.getCurrentTileY() != desired_tile.y)
           && !KILL_SWITCH)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // TODO
    // We could possibly trigger a scan in here but maybe we should create a scan tile
    // function in planner to search tile for one of the 3 objects
    while (sensor_readings.getDetectionBit() == 0x00 && !KILL_SWITCH)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}