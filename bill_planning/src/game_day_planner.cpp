#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_planning/sensor_readings.hpp"
#include <math.h>
#include "bill_planning/position.hpp"
#include "bill_msgs/Position.h"
#include <cstddef>
#include <iostream>
#include <utility>
#include <thread>
#include <chrono>
#include <bill_planning/planner.hpp>
#include <cmath>
#include <vector>
#include "bill_msgs/Survivor.h"

// CALLBACKS
void positionCallback(const bill_msgs::Position::ConstPtr& msg);
void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg);
void leftUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg);
void rightUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg);
void fireCallbackFront(const std_msgs::Bool::ConstPtr& msg);
void fireCallbackLeft(const std_msgs::Bool::ConstPtr& msg);
void fireCallbackRight(const std_msgs::Bool::ConstPtr& msg);
void hallCallback(const std_msgs::Bool::ConstPtr& msg);
void survivorsCallback(const bill_msgs::Survivor::ConstPtr& msg);

// HELPER FUNCTIONS
void fireOut();
void findClearPathFwd();
void completeStraightLineSearch();
void driveToDesiredPoints();
void robotPerformanceThread(int n);
TilePosition tileFromPoint(int x_pos, int y_pos);
void waitToHitTile();
void completeTSearch();
void driveToFlame();
void driveHome();
void driveToLargeBuilding();
void conductGridSearch();
void waitForPlannerScan();
void runInitialSearch();
void startSearchYDependent();
void startSearchXDependent();
void completeSearchYDependent();
void completeSearchXDependent();

SensorReadings sensor_readings;

int found_fire_front = 0;
int found_fire_left = 0;
int found_fire_right = 0;
unsigned char start_course = 0x00;
Planner planner;

// FLAGS
bool _cleared_fwd = false;
bool _driven_fwd = false;
bool _found_hall = true; // TODO SET TO  WHEN ACTUALLY ATTACHED false;
bool KILL_SWITCH = false; 

// POSITION
TilePosition desired_tile(0,0);
TilePosition large_building_tile(-1,-1);

int desired_heading = 90;

// CONSTANTS
const int FULL_COURSE_DETECTION_LENGTH = 1.60;
const int FULL_COURSE_SIDE_ULTRAS = 1.5;
const int FIRE_SCAN_ANGLE = 20;
const float DELTA = 7; //cm
const float TILE_WIDTH = 0.3;
const float TILE_HEIGHT = 0.3;
const float POSITION_ACCURACY_BUFFER = 0.075;
// There is a buffer in the robot response time so let's be a bit more generous here. In degrees
const float HEADING_ACCURACY_BUFFER = 4.0;
// There is a buffer in the robot response time so let's be a bit more generous here. In cm
const float OBSTACLE_THRESHOLD = 3.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "game_day_planner");
    ros::NodeHandle nh;

    // Subscribing to Topics
    ros::Subscriber sub_odom = nh.subscribe("position", 1, positionCallback);
    ros::Subscriber sub_fire = nh.subscribe("fire", 1, fireCallbackFront);
    ros::Subscriber sub_fire_left = nh.subscribe("fire_left", 1, fireCallbackLeft);
    ros::Subscriber sub_fire_right = nh.subscribe("fire_right", 1, fireCallbackRight);
    ros::Subscriber sub_ultrasonic = nh.subscribe("ultra_front", 1, frontUltrasonicCallback);
    ros::Subscriber sub_ultrasonic_right = nh.subscribe("ultra_right", 1, rightUltrasonicCallback);
    ros::Subscriber sub_ultrasonic_left = nh.subscribe("ultra_left", 1, leftUltrasonicCallback);
    ros::Subscriber sub_food = nh.subscribe("food", 1, hallCallback);
    ros::Subscriber sub_survivors = nh.subscribe("survivors", 1, survivorsCallback);

    ros::Publisher motor_pub = nh.advertise<bill_msgs::MotorCommands>("motor_cmd", 100, true);
    ros::Publisher fan_pub = nh.advertise<std_msgs::Bool>("fan", 100);
    ros::Publisher state_pub;  //= nh.advertise<bill_msgs::State>("state", 100);
    ros::Publisher led_pub = nh.advertise<std_msgs::Bool>("led", 100);

    planner.setPubs(motor_pub, fan_pub, led_pub);

    sensor_readings.setCurrentState(STATE::FINDING_T_SEARCH_TILE);
    std::thread robot_execution_thread(robotPerformanceThread, 1);
    ros::spin();
    KILL_SWITCH = true;
    robot_execution_thread.join();
    return 0;
}

void robotPerformanceThread(int n)
{
    ROS_INFO("RUNNING SEPARATE THREAD: %i", n);
    // WAIT ON DATA FROM EACH ULTRASONIC SENSOR
    while (!sensor_readings.getStartRobotPerformanceThread() && !KILL_SWITCH)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // We can get this based off of chosed setup ask seamus
    sensor_readings.setHomeTile(sensor_readings.getCurrentTileX(),sensor_readings.getCurrentTileY());
    
    ROS_INFO("Current tile: (%i,%i)", sensor_readings.getCurrentTileX(), sensor_readings.getCurrentTileX());
    runInitialSearch();

    sensor_readings.setCurrentState(STATE::FLAME_SEARCH);


    ROS_INFO("DRIVING TO FLAME");
    driveToFlame();

    ROS_INFO("DRIVING TO ALL OTHER SAVED POITNS");
    driveToDesiredPoints();

    // Conduct our grid search
    if (!_found_hall)
    {
        sensor_readings.setCurrentState(STATE::HALL_SEARCH);
        conductGridSearch();
        // TODO  If we ever get hall data
        //      Cancel Grid search
    }

    sensor_readings.setCurrentState(STATE::BUILDING_SEARCH);
    // Drive to the large building again b/c we must have found the hall
    driveToLargeBuilding();

    // Returning Home
    sensor_readings.setCurrentState(STATE::RETURN_HOME);
    driveHome();
}

void positionCallback(const bill_msgs::Position::ConstPtr& msg)
{
    sensor_readings.setCurrentHeading(msg->heading);

    // Convert units to tiles instead of meters
    float currentXTileCoordinate = msg->x / TILE_WIDTH;
    float currentYTileCoordinate = msg->y / TILE_HEIGHT;

    float currentWholeX;
    float currentWholeY;

    float localTileXCoverage = std::modf(currentXTileCoordinate, &currentWholeX);
    float localTileYCoverage = std::modf(currentYTileCoordinate, &currentWholeY);

    bool isOnXTile = fabs(0.5 - fabs(localTileXCoverage)) < (POSITION_ACCURACY_BUFFER / TILE_WIDTH);
    bool isOnYTile = fabs(0.5 - fabs(localTileYCoverage)) < (POSITION_ACCURACY_BUFFER / TILE_WIDTH);

    if (isOnXTile)
    {
        sensor_readings.setCurrentTileX((int)currentWholeX);
    }

    if (isOnYTile)
    {
        sensor_readings.setCurrentTileY((int)currentWholeY);
    }

    // If there is a valid target heading that means we are turning
    if (sensor_readings.getTargetHeading() >= 0)
    {
        if (fabs(sensor_readings.getTargetHeading() - sensor_readings.getCurrentHeading()) < HEADING_ACCURACY_BUFFER)
        {
            //ROS_INFO("Arrived at target heading %i, publishing drive", sensor_readings.getTargetHeading());
            planner.publishStop();
            planner.publishDrive(sensor_readings.getCurrentHeading(), 0.4);

            // publish an invalid target heading
            sensor_readings.setTargetHeading(-1);
        }
    }
    // Other wise we must be driving
    else
    {
        // Check for obstacles
        if (sensor_readings.getUltraFwd() < OBSTACLE_THRESHOLD && (sensor_readings.getDetectionBit() != 0))
        {
            // Brake immediately
            planner.publishStop();

            // Should move into obstacle avoidance
            if (sensor_readings.getUltraLeft() < OBSTACLE_THRESHOLD)
            {
                planner.driveAroundObstacle(sensor_readings);

                // Buzz, because why not
                planner.signalComplete();
            }

            return;
        }
        // Regular free driving
        else if (sensor_readings.getTargetTileX() == sensor_readings.getCurrentTileX() && sensor_readings.getTargetTileY() == sensor_readings.getCurrentTileY())
        {
            //ROS_INFO("Arrived at target point: %i, %i", sensor_readings.getTargetTileX(), sensor_readings.getTargetTileY());
            // We have arrived at our current target point
            planner.ProcessNextDrivePoint(sensor_readings);
        }
    }
}

void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    sensor_readings.setUltraFwd(msg->data);
    
    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x01;
    if(!sensor_readings.getStartRobotPerformanceThread()
       && 0x07 - start_course == 0x00)
    {
        sensor_readings.setStartRobotPerformanceThread(true);
    }
}

void leftUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if (sensor_readings.getCurrentState() == STATE::INIT_SEARCH
        && (sensor_readings.getUltraLeft() - msg->data) > DELTA)
    {
        int h = sensor_readings.getCurrentHeading();
        if (std::abs(h-180) < HEADING_ACCURACY_BUFFER)
        {
            //Then we must be travelling parallel to x axis
            int signal_point_y = (int)(sensor_readings.getCurrentPositionY() * 100 - msg->data);
            int signal_point_x = (int)(sensor_readings.getCurrentPositionX() * 100);
            sensor_readings.pointsOfInterestEmplace(tileFromPoint(signal_point_x, signal_point_y));
        }
        else if (std::abs(h-360) < HEADING_ACCURACY_BUFFER || h < HEADING_ACCURACY_BUFFER)
        {
            //Then we must be travelling parallel to x axis
            int signal_point_y = (int)(sensor_readings.getCurrentPositionY() * 100 + msg->data);
            int signal_point_x = (int)(sensor_readings.getCurrentPositionX() * 100);
            sensor_readings.pointsOfInterestEmplace(tileFromPoint(signal_point_x, signal_point_y));
        }
        else if (std::abs(h-90) < HEADING_ACCURACY_BUFFER)
        {        
            int signal_point_x = (int)(sensor_readings.getCurrentPositionX() * 100 - msg->data);
            int signal_point_y = (int)(sensor_readings.getCurrentPositionY() * 100);
            sensor_readings.pointsOfInterestEmplace(tileFromPoint(signal_point_x, signal_point_y));
        }
        else if (std::abs(h-270) < HEADING_ACCURACY_BUFFER)
        {        
            int signal_point_x = (int)(sensor_readings.getCurrentPositionX() * 100 + msg->data);
            int signal_point_y = (int)(sensor_readings.getCurrentPositionY() * 100);
            sensor_readings.pointsOfInterestEmplace(tileFromPoint(signal_point_x, signal_point_y));
        }
    }

    sensor_readings.setUltraLeft(msg->data);

    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x02;
    if(!sensor_readings.getStartRobotPerformanceThread()
       && 0x07 - start_course == 0x00)
    {
        sensor_readings.setStartRobotPerformanceThread(true);
    }
}

void rightUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if (sensor_readings.getCurrentState() == STATE::INIT_SEARCH
        && (sensor_readings.getUltraRight() - msg->data) > DELTA)
    {
        int h = sensor_readings.getCurrentHeading();
        if (std::abs(h-180) < HEADING_ACCURACY_BUFFER)
        {
            //Then we must be travelling parallel to x axis
            int signal_point_y = (int)(sensor_readings.getCurrentPositionY() * 100 + msg->data);
            int signal_point_x = (int)(sensor_readings.getCurrentPositionX() * 100);
            sensor_readings.pointsOfInterestEmplace(tileFromPoint(signal_point_x, signal_point_y));
        }
        else if (std::abs(h-360) < HEADING_ACCURACY_BUFFER || h < HEADING_ACCURACY_BUFFER)
        {
            //Then we must be travelling parallel to x axis
            int signal_point_y = (int)(sensor_readings.getCurrentPositionY() * 100 - msg->data);
            int signal_point_x = (int)(sensor_readings.getCurrentPositionX() * 100);
            sensor_readings.pointsOfInterestEmplace(tileFromPoint(signal_point_x, signal_point_y));
        }
        else if (std::abs(h-90) < HEADING_ACCURACY_BUFFER)
        {        
            int signal_point_x = (int)(sensor_readings.getCurrentPositionX() * 100 + msg->data);
            int signal_point_y = (int)(sensor_readings.getCurrentPositionY() * 100);
            sensor_readings.pointsOfInterestEmplace(tileFromPoint(signal_point_x, signal_point_y));
        }
        else if (std::abs(h-270) < HEADING_ACCURACY_BUFFER)
        {        
            int signal_point_x = (int)(sensor_readings.getCurrentPositionX() * 100 - msg->data);
            int signal_point_y = (int)(sensor_readings.getCurrentPositionY() * 100);
            sensor_readings.pointsOfInterestEmplace(tileFromPoint(signal_point_x, signal_point_y));
        }
    }

    sensor_readings.setUltraRight(msg->data);

    //WHEN FRONT, RIGHT AND LEFT EACH HAVE VALID DATA:
    start_course = start_course ^ 0x04;
    if(!sensor_readings.getStartRobotPerformanceThread()
       && 0x07 - start_course == 0x00)
    {
        sensor_readings.setStartRobotPerformanceThread(true);
    }
}

void fireCallbackFront(const std_msgs::Bool::ConstPtr& msg)
{
    if (sensor_readings.getCurrentState() == STATE::INIT_SEARCH
        && msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        if (found_fire_front < 3)
        {
            found_fire_front++;
            sensor_readings.setDetectedFireFwd(false);
            fireOut();
        }
        else
        {
            sensor_readings.setDetectedFireFwd(true);
        }
    }
    else
    {
        found_fire_front = 0;
        sensor_readings.setDetectedFireFwd(false);
    }
}

void fireCallbackLeft(const std_msgs::Bool::ConstPtr& msg)
{
    if (sensor_readings.getFlameTileX() != -1 || sensor_readings.getFlameTileY() != -1)
    {
        return;
    }

    if (sensor_readings.getCurrentState() == STATE::INIT_SEARCH
        && msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        if (found_fire_left < 3)
        {
            found_fire_left++;
            sensor_readings.setDetectedFireLeft(false);
        }
        else
        {
            sensor_readings.setDetectedFireLeft(true);
            sensor_readings.updateFlameTileFromLastSavedPoint(true);
        }
    }
    else
    {
        found_fire_left = 0;
        sensor_readings.setDetectedFireLeft(false);
    }
}

void fireCallbackRight(const std_msgs::Bool::ConstPtr& msg)
{
    if (sensor_readings.getFlameTileX() != -1 || sensor_readings.getFlameTileY() != -1)
    {
        return;
    }
    
    if (msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        if (found_fire_right < 3)
        {
            found_fire_right++;
            sensor_readings.setDetectedFireRight(false);
        }
        else
        {
            sensor_readings.setDetectedFireRight(true);
            sensor_readings.updateFlameTileFromLastSavedPoint(false);
        }
    }
    else
    {
        found_fire_right = 0;
        sensor_readings.setDetectedFireRight(false);
    }
}

void hallCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(!_found_hall && msg->data)
    {
        // TODO MAKE SURE THIS DOESN'T DO ANYTHING IF NOT CURRENTLY GRID SEARCHING?
        planner.cancelGridSearch(sensor_readings);
        _found_hall = true;
        planner.signalComplete();
    }
}

void survivorsCallback(const bill_msgs::Survivor::ConstPtr& msg)
{
    bool multiple = msg->data == msg->SURVIVOR_MULTIPLE;
    bool single = msg->data == msg->SURVIVOR_SINGLE;

    if (multiple || single)
    {
        if (multiple)
        {
            sensor_readings.setDetectionBit(0x03);
        }

        if (single)
        {
            sensor_readings.setDetectionBit(0x02);
        }

        if (planner.getIsScanning())
        {
            ROS_INFO("Found a building!");
            planner.signalComplete();
            planner.publishStop();
        }
    }
    else
    {
        sensor_readings.setDetectionBit(0);
    }
}

bool shouldKeepTurning()
{
    if (fabs(desired_heading - sensor_readings.getCurrentHeading()) < HEADING_ACCURACY_BUFFER)
    {
        planner.publishStop();
        return false;
    }
    else
    {
        return true;
    }
}

void fireOut()
{
    bool initialCall = true;
    bool check_temp_heading = true;
    bool set_desired_heading = false;
    int temp_desired_heading = 0;

    do
    {
        if (initialCall || sensor_readings.getDetectedFireFwd())
        {
            planner.putOutFire();
            desired_heading = (sensor_readings.getCurrentHeading() + 2 * FIRE_SCAN_ANGLE + 360) % 360;
            temp_desired_heading = (sensor_readings.getCurrentHeading() - FIRE_SCAN_ANGLE + 360) % 360;

            planner.publishTurn(temp_desired_heading);

            initialCall = false;
            check_temp_heading = true;
            set_desired_heading = false;
        }

        while(check_temp_heading
              && !(fabs(temp_desired_heading - sensor_readings.getCurrentHeading()) < HEADING_ACCURACY_BUFFER)
              && !KILL_SWITCH)
        {
            if(sensor_readings.getDetectedFireFwd())
            {
                planner.putOutFire();
                desired_heading = (sensor_readings.getCurrentHeading() + 2 * FIRE_SCAN_ANGLE + 360) % 360;
                temp_desired_heading = (sensor_readings.getCurrentHeading() - FIRE_SCAN_ANGLE + 360) % 360;
                planner.publishTurn(temp_desired_heading);
            }
            set_desired_heading = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        check_temp_heading = false; 
        if (!set_desired_heading)
        {
            planner.publishTurn(desired_heading);
            set_desired_heading = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while(shouldKeepTurning() && !KILL_SWITCH);

    sensor_readings.setCurrentState(STATE::BUILDING_SEARCH);
}

void driveToDesiredPoints()
{
    while (!sensor_readings.pointsOfInterestEmpty() && !KILL_SWITCH)
    {
        TilePosition newTarget = sensor_readings.pointsOfInterestFront();
        sensor_readings.pointsOfInterestPop();

        if (newTarget.x < 0 || newTarget.y < 0)
        {
            //GARBAGE DATA, CONTINUE
            continue;
        }

        //sensor_readings.setTargetPoint(newTarget.x, newTarget.y);

        desired_tile.x = sensor_readings.getTargetTileX();
        desired_tile.y = sensor_readings.getTargetTileY();

        planner.publishDriveToTile(sensor_readings,
            desired_tile.x,
            desired_tile.y, 0.4, true);
        
        waitForPlannerScan();

        while (sensor_readings.getDetectionBit() == 0x00
            && !KILL_SWITCH)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (sensor_readings.getDetectedFireFwd())
        {
            fireOut();
        }
        else if (sensor_readings.getCurrentState() == STATE::BUILDING_SEARCH)
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
            sensor_readings.pointsOfInterestEmplace(TilePosition(sensor_readings.getTargetTileX(), sensor_readings.getTargetTileY()));
        }

        sensor_readings.setDetectionBit(0x00);
    }
}

void conductGridSearch()
{
    planner.gridSearch(sensor_readings);
}

void driveHome()
{
    desired_tile.x = sensor_readings.getHomeTileX();
    desired_tile.y = sensor_readings.getHomeTileY();

    planner.publishDriveToTile(sensor_readings,
        desired_tile.x,
        desired_tile.y, 0.4);
    waitToHitTile();
}

void driveToLargeBuilding()
{
    //sensor_readings.setTargetPoint(large_building_tile.x, large_building_tile.y);

    desired_tile.x = large_building_tile.x;
    desired_tile.y = large_building_tile.y;

    planner.publishDriveToTile(sensor_readings,
        desired_tile.x,
        desired_tile.y, 0.4, true);
    waitForPlannerScan();

    while (sensor_readings.getDetectionBit() == 0x00 && !KILL_SWITCH)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void waitToHitTile()
{
    while((planner.is_moving 
        || desired_tile.x != sensor_readings.getCurrentTileX() ||
        desired_tile.y != sensor_readings.getCurrentTileY()) 
        && !KILL_SWITCH)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void completeTSearch()
{
    ROS_INFO("FOUND LESS THAN 3 POINTS OF INTEREST");

    desired_tile.x = sensor_readings.getCurrentTileX();
    desired_tile.y = sensor_readings.freeRowTile();

    planner.publishDriveToTile(
        sensor_readings,
        desired_tile.x,
        desired_tile.y, 0.4);
    waitToHitTile();

    desired_tile.x = 0;
    desired_tile.y = sensor_readings.getCurrentTileY();

    planner.publishDriveToTile(
        sensor_readings,
        desired_tile.x,
        desired_tile.y, 0.4);
    waitToHitTile();

    desired_tile.x = 5;

    planner.publishDriveToTile(
        sensor_readings,
        desired_tile.x,
        desired_tile.y, 0.4);
    waitToHitTile();
}

void driveToFlame()
{
    if(sensor_readings.getFlameTileX() >= 0 && sensor_readings.getFlameTileY() >= 0)
    {
        ROS_INFO("Have a flam tile stored");
        desired_tile.x = sensor_readings.getFlameTileX();
        desired_tile.y = sensor_readings.getFlameTileY();

        planner.publishDriveToTile(
            sensor_readings,
            desired_tile.x,
            desired_tile.y, 0.4);

        waitForPlannerScan();

        if (sensor_readings.getDetectedFireFwd())
        {
            //PUT OUT FLAME
            fireOut();
        }
    }
}

void waitForPlannerScan()
{
    int i = 0;
    while(!planner.getIsScanning() && i < 200)
    {
        i++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    i = 0;
    while(planner.getIsScanning() && i < 200)
    {
        i++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// THIS SHOULD BE PROVIDED IN CM AND TRUNCATED
TilePosition tileFromPoint(int x_pos, int y_pos)
{
    if (x_pos < 0 || x_pos > 6 || y_pos < 0 || y_pos > 6)
    {
        ROS_INFO("TRIED TO CONVERT A TILE OUT OF RANGE");
        return TilePosition(-1,-1);
    }

    //Truncated value should be the tile position
    int x = x_pos/30;
    int y = y_pos/30;
    return TilePosition(x,y);
}

void runInitialSearch()
{
    int x = sensor_readings.getCurrentTileX();
    int y = sensor_readings.getCurrentTileY();
    ROS_INFO("Current Before Running initial search tile: (%i, %i)", x, y);
    if ((x == 3 && y == 0)
        || (x == 2 && y == 5))
    {
        // TOP OR BOTTOM
        startSearchXDependent();
        
        ROS_INFO("Found %i POIs", sensor_readings.pointsOfInterestSize());
        
        if(sensor_readings.pointsOfInterestSize() < 3)
        {
            sensor_readings.setCurrentState(STATE::FINDING_T_SEARCH_TILE);
            completeSearchXDependent();
        }
    }
    else if ((x == 0 && y == 2) || (x == 5 && y == 3))
    {
        // LEFT OR RIGHT
        ROS_INFO("Running y dependant");
        startSearchYDependent();
        ROS_INFO("Found %i POIs", sensor_readings.pointsOfInterestSize());

        if(sensor_readings.pointsOfInterestSize() < 3)
        {
            sensor_readings.setCurrentState(STATE::FINDING_T_SEARCH_TILE);
            completeSearchYDependent();
        }
    }
    else
    {
        ROS_INFO("NOT RUNNING ANYTHING");
        ROS_WARN("OUR CURRENT POSITION IS WRONG AND WE CAN'T START");
    }
}

void startSearchXDependent()
{
    ROS_INFO("Starting search x dependant");
    int y = sensor_readings.getCurrentTileY();
    ROS_INFO("Current tile Y on straight line search is %i", y);
    TilePosition poi[3] = {TilePosition(3,y), TilePosition(4,y), TilePosition(0,y)};
    for(int i = 0; i < 3; i++)
    {
        desired_tile.x = poi[i].x;
        desired_tile.y = poi[i].y;

        if (desired_tile.x == sensor_readings.getCurrentTileX() && desired_tile.y == sensor_readings.getCurrentTileY())
        {
            continue;
        }

        ROS_INFO("Driving to tile x = %i, y = %i", desired_tile.x, desired_tile.y);
        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.4);
        waitToHitTile();

        desired_heading = y == 5 ? 270 : 90;
        planner.publishTurn(desired_heading);

        while (shouldKeepTurning() && !KILL_SWITCH)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        float uf = sensor_readings.getUltraFwd();
        ROS_INFO("Ultra fwd is %f", uf);
        if(uf >= FULL_COURSE_DETECTION_LENGTH && uf <= 200)
        {
            ROS_INFO("Found a clear path fwd");
            break;
        }
    }

    ROS_INFO("Found/Defaulted a path and driving forward now");
    ROS_INFO("Setting state to init search");
    sensor_readings.setCurrentState(STATE::INIT_SEARCH);
    
    desired_tile.x = sensor_readings.getCurrentTileX();
    desired_tile.y = sensor_readings.getCurrentTileY() == 0 ? 5 : 0;

    ROS_INFO("Driving to tile x = %i, y = %i", desired_tile.x, desired_tile.y);
    planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.4);
    waitToHitTile();
    ROS_INFO("Finished straight line search");
}

void startSearchYDependent()
{
    int x = sensor_readings.getCurrentTileX();
    TilePosition poi[3] = {TilePosition(x,1), TilePosition(x,3), TilePosition(x,5)};
    for(int i = 0; i < 3; i++)
    {
        desired_tile.x = poi[i].x;
        desired_tile.y = poi[i].y;

        if (desired_tile.x == sensor_readings.getCurrentTileX() && desired_tile.y == sensor_readings.getCurrentTileY())
        {
            continue;
        }


        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.4);
        waitToHitTile();

        desired_heading = x == 0 ? 0 : 180;
        planner.publishTurn(desired_heading);

        while (shouldKeepTurning() && !KILL_SWITCH)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if(sensor_readings.getUltraFwd() >= FULL_COURSE_DETECTION_LENGTH)
        {
            break;
        }
    }
    sensor_readings.setCurrentState(STATE::INIT_SEARCH);

    desired_tile.x = sensor_readings.getCurrentTileX() == 0 ? 5 : 0;
    desired_tile.y = sensor_readings.getCurrentTileY();

    planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.4);
    waitToHitTile();
}

void completeSearchXDependent()
{
    int x = sensor_readings.getCurrentTileX();
    TilePosition poi[3] = {TilePosition(x,3), TilePosition(x,1), TilePosition(x,5)};
    for(int i = 0; i < 3; i++)
    {
        desired_tile.x = poi[i].x;
        desired_tile.y = poi[i].y;

        if (desired_tile.x == sensor_readings.getCurrentTileX() &&
            desired_tile.y == sensor_readings.getCurrentTileY())
        {
            continue;
        }

        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.4);
        waitToHitTile();

        if(sensor_readings.getUltraRight() + sensor_readings.getUltraLeft() >= FULL_COURSE_SIDE_ULTRAS)
        {
            break;
        }
    }

    if ((std::abs(sensor_readings.getCurrentHeading() - 90) < HEADING_ACCURACY_BUFFER
        && sensor_readings.getUltraRight() >= sensor_readings.getUltraLeft())
        || (std::abs(sensor_readings.getCurrentHeading() - 270) < HEADING_ACCURACY_BUFFER
        && sensor_readings.getUltraRight() <= sensor_readings.getUltraLeft()))
    {
        desired_tile.x = 5;
        desired_tile.y = sensor_readings.getCurrentTileY();
        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.3);
        waitToHitTile();
        

        sensor_readings.setCurrentState(STATE::INIT_SEARCH);
        desired_tile.x = 0;
        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.3);
        waitToHitTile();
    }
    else if ((std::abs(sensor_readings.getCurrentHeading() - 90) < HEADING_ACCURACY_BUFFER
        && sensor_readings.getUltraRight() <= sensor_readings.getUltraLeft())
        || (std::abs(sensor_readings.getCurrentHeading() - 270) < HEADING_ACCURACY_BUFFER
        && sensor_readings.getUltraRight() >= sensor_readings.getUltraLeft()))
    {
        desired_tile.x = 0;
        desired_tile.y = sensor_readings.getCurrentTileY();
        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.3);
        waitToHitTile();

        sensor_readings.setCurrentState(STATE::INIT_SEARCH);
        desired_tile.x = 5;
        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.3);
        waitToHitTile();
    }
    else
    {
        ROS_WARN("SOMETHING WENT SUPER WRONG COMPLETING T SEARCH");
    }
}

void completeSearchYDependent()
{
    int y = sensor_readings.getCurrentTileY();
    TilePosition poi[3] = {TilePosition(3, y), TilePosition(4,y), TilePosition(0,y)};
    for(int i = 0; i < 3; i++)
    {
        desired_tile.x = poi[i].x;
        desired_tile.y = poi[i].y;

        if (desired_tile.x == sensor_readings.getCurrentTileX() && 
            desired_tile.y == sensor_readings.getCurrentTileY())
        {
            continue;
        }

        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.4);
        waitToHitTile();

        if(sensor_readings.getUltraRight() + sensor_readings.getUltraLeft() >= FULL_COURSE_SIDE_ULTRAS)
        {
            break;
        }
    }

    int ch = sensor_readings.getCurrentHeading();
    if (((std::abs(ch-360) < HEADING_ACCURACY_BUFFER || ch < HEADING_ACCURACY_BUFFER)
        && sensor_readings.getUltraRight() >= sensor_readings.getUltraLeft())
        || (std::abs(sensor_readings.getCurrentHeading() - 180) < HEADING_ACCURACY_BUFFER
        && sensor_readings.getUltraRight() <= sensor_readings.getUltraLeft()))
    {
        desired_tile.x = sensor_readings.getCurrentTileX();
        desired_tile.y = 5;
        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.3);
        waitToHitTile();
        
        sensor_readings.setCurrentState(STATE::INIT_SEARCH);
        desired_tile.x = 0;
        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.3);
        waitToHitTile();
    }
    else if (((std::abs(ch-360) < HEADING_ACCURACY_BUFFER || ch < HEADING_ACCURACY_BUFFER)
        && sensor_readings.getUltraRight() <= sensor_readings.getUltraLeft())
        || (std::abs(sensor_readings.getCurrentHeading() - 180) < HEADING_ACCURACY_BUFFER
        && sensor_readings.getUltraRight() >= sensor_readings.getUltraLeft()))
    {
        desired_tile.x = sensor_readings.getCurrentTileX();
        desired_tile.y = 0;

        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.3);
        waitToHitTile();

        sensor_readings.setCurrentState(STATE::INIT_SEARCH);
        desired_tile.x = 5;
        planner.publishDriveToTile(sensor_readings, desired_tile.x, desired_tile.y, 0.3);
        waitToHitTile();
    }
    else
    {
        ROS_WARN("SOMETHING WENT SUPER WRONG COMPLETING T SEARCH");
    }
}






//OLD CODE -- MIGHT NEED -- DON'T TOUCH - NEVER CALLED
void completeStraightLineSearch()
{
    ROS_INFO("Findings Clear Path Fwd");
    findClearPathFwd(); //TODO UNCOMMENT THIS
    ROS_INFO("Found Clear Path Fwd");

    desired_tile.x = 3;// TODO CHANGE TO THIS sensor_readings.getCurrentTileX();
    desired_tile.y = 5;

    planner.publishDriveToTile(sensor_readings,
        desired_tile.x,
        desired_tile.y, 0.2);
    ROS_INFO("Published drive");
    waitToHitTile();
    ROS_INFO("Hit Tile");
}


void findClearPathFwd()
{
    if(!_cleared_fwd 
        && sensor_readings.getUltraFwd() >= FULL_COURSE_DETECTION_LENGTH)
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
                desired_tile.x = sensor_readings.getCurrentTileX() + increment;
                desired_tile.y = 0;

                planner.publishDriveToTile(
                        sensor_readings,
                        desired_tile.x,
                        desired_tile.y, 0.4);
                // THE ULTRASONIC CALLBACK WILL BE IN CHARGE OF SAVING THE POINT OF INTEREST
            }
            else if (temp_ultra > FULL_COURSE_DETECTION_LENGTH)
            {
                _cleared_fwd = true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        desired_heading = 90;
    }
}
