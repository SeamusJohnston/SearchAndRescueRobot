#include "bill_planning/sensor_readings.hpp"

SensorReadings::SensorReadings()
{
    points_of_interest = std::queue<TilePosition>();
}

void SensorReadings::setStartRobotPerformanceThread(bool val)
{
    _start_robot_performance_thread_mutex.lock();
    _start_robot_performance_thread = val;
    _start_robot_performance_thread_mutex.unlock();
}

bool SensorReadings::getStartRobotPerformanceThread()
{
    //std::lock_guard<std::mutex> guard(_start_robot_performance_thread_mutex);
    return _start_robot_performance_thread;
}

void SensorReadings::setUltraFwd(float val)
{
    //std::lock_guard<std::mutex> guard(_ultra_fwd_mutex);
    _ultra_fwd = val;
}

float SensorReadings::getUltraFwd()
{
    //std::lock_guard<std::mutex> guard(_ultra_fwd_mutex);
    return _ultra_fwd;
}

void SensorReadings::setUltraLeft(float val)
{
    //std::lock_guard<std::mutex> guard(_ultra_left_mutex);
    _ultra_left = val;
}

float SensorReadings::getUltraLeft()
{
    //std::lock_guard<std::mutex> guard(_ultra_left_mutex);
    return _ultra_left;
}

void SensorReadings::setUltraRight(float val)
{
    //std::lock_guard<std::mutex> guard(_ultra_right_mutex);
    _ultra_right = val;
}

float SensorReadings::getUltraRight()
{
    //std::lock_guard<std::mutex> guard(_ultra_right_mutex);
    return _ultra_right;
}

void SensorReadings::setDetectedFire(bool val)
{
    //std::lock_guard<std::mutex> guard(_detected_fire_mutex);
    _detected_fire = val;
}

bool SensorReadings::getDetectedFire()
{
    //std::lock_guard<std::mutex> guard(_detected_fire_mutex);
    return _detected_fire;
}

void SensorReadings::setCurrentHeading(int val)
{
    //std::lock_guard<std::mutex> guard(_current_heading_mutex);
    _current_heading = val;
}

int SensorReadings::getCurrentHeading()
{
    //std::lock_guard<std::mutex> guard(_current_heading_mutex);
    return _current_heading;
}

void SensorReadings::setCurrentTileX(int val)
{
    // std::lock_guard<std::mutex> guard(_current_tile_mutex);
    _current_tile.x = val;
}

void SensorReadings::setCurrentTileY(int val)
{
    // std::lock_guard<std::mutex> guard(_current_tile_mutex);
    _current_tile.y = val;
}

int SensorReadings::getCurrentTileX()
{
    // std::lock_guard<std::mutex> guard(_current_tile_mutex);
    return _current_tile.x;
}

int SensorReadings::getCurrentTileY()
{
    // std::lock_guard<std::mutex> guard(_current_tile_mutex);
    return _current_tile.y;
}

void SensorReadings::setCurrentState(STATE val)
{
    // std::lock_guard<std::mutex> guard(_current_state_mutex);
    _current_state = val;
}

STATE SensorReadings::getCurrentState()
{
    // std::lock_guard<std::mutex> guard(_current_state_mutex);
    return _current_state;
}

void SensorReadings::setDetectionBit(unsigned char val)
{
    // std::lock_guard<std::mutex> guard(_detection_bit_mutex);
    _detection_bit = val;
}

unsigned char SensorReadings::getDetectionBit()
{
    //std::lock_guard<std::mutex> guard(_detection_bit_mutex);
    return _detection_bit;
}sss