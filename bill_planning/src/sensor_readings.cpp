#include "bill_planning/sensor_readings.hpp"

SensorReadings::SensorReadings()
{
    current_tile = TilePosition(0,0);
    current_heading = 90;
    detected_fire = false;
    start_robot_performance_thread = false;
    ultra_fwd = -500;
    ultra_left = -500;
    ultra_right = -500;
    detection_bit = 0x00;
    currentTargetPoint(0,0);
    current_state = STATE::INIT_SEARCH;
    points_of_interest = std::queue<TilePosition>();
}

void SensorReadings::setStartRobotPerformanceThread(bool val)
{
    std::lock_guard<std::mutex> lock(_start_robot_performance_thread_mutex);
    _start_robot_performance_thread = val;
}

bool SensorReadings::getStartRobotPerformanceThread()
{
    std::lock_guard<std::mutex> lock(_start_robot_performance_thread_mutex);
    return _start_robot_performance_thread;
}

void SensorReadings::setUltraFwd(float val)
{
    std::lock_guard<std::mutex> lock(_ultra_fwd_mutex);
    _ultra_fwd = val;
}

float SensorReadings::getUltraFwd()
{
    std::lock_guard<std::mutex> lock(_ultra_fwd_mutex);
    return _ultra_fwd;
}

void SensorReadings::setUltraLeft(float val)
{
    std::lock_guard<std::mutex> lock(_ultra_left_mutex);
    _ultra_left = val;
}

float SensorReadings::getUltraLeft()
{
    std::lock_guard<std::mutex> lock(_ultra_left_mutex);
    return _ultra_left;
}

void SensorReadings::setUltraRight(float val)
{
    std::lock_guard<std::mutex> lock(_ultra_right_mutex);
    _ultra_right = val;
}

float SensorReadings::getUltraRight()
{
    std::lock_guard<std::mutex> lock(_ultra_right_mutex);
    return _ultra_right;
}

void setDetectedFire(bool val)
{
    std::lock_guard<std::mutex> lock(_detected_fire_mutex);
    _detected_fire = val;
}

bool getDetectedFire()
{
    std::lock_guard<std::mutex> lock(_detected_fire_mutex);
    return _detected_fire;
}

void setCurrentHeading(int val)
{
    std::lock_guard<std::mutex> lock(_current_heading_mutex);
    _current_heading = val;
}

int getCurrentHeading()
{
    std::lock_guard<std::mutex> lock(_current_heading_mutex);
    return _current_heading;
}

void setCurrentTileX(int val)
{
    std::lock_guard<std::mutex> lock(_current_tile_mutex);
    _current_tile.x = val;
}

void setCurrentTileY(int val)
{
    std::lock_guard<std::mutex> lock(_current_tile_mutex);
    _current_tile.y = val;
}

int getCurrentTileX()
{
    std::lock_guard<std::mutex> lock(_current_tile_mutex);
    return _current_tile.x;
}

int getCurrentTileY()
{
    std::lock_guard<std::mutex> lock(_current_tile_mutex);
    return _current_tile.y;
}

void setCurrentState(STATE val)
{
    std::lock_guard<std::mutex> lock(_current_state_mutex);
    _current_state = val;
}

STATE getCurrentState()
{
    std::lock_guard<std::mutex> lock(_current_state_mutex);
    return _current_state;
}

void setDetectionBit(unsigned char val)
{
    std::lock_guard<std::mutex> lock(_detection_bit_mutex);
    _detection_bit = val;
}

unsigned char getDetectionBit()
{
    std::lock_guard<std::mutex> lock(_detection_bit_mutex);
    return _detection_bit;
}