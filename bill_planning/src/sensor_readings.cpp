#include "bill_planning/sensor_readings.hpp"

SensorReadings::SensorReadings()
{
    //IF YOU DON'T DECLARE STATIC MEMBERS WITH A VALUE, IT WONT BUILD
    //TODO CHECK IF WE CAN THROW THIS IN sensor_readings.cpp
    TilePosition SensorReadings::current_tile(0,0);
    int SensorReadings::current_heading = 90;
    bool SensorReadings::detected_fire = false;
    bool SensorReadings::start_robot_performance_thread = false;
    float SensorReadings::ultra_fwd = -500;
    float SensorReadings::ultra_left = -500;
    float SensorReadings::ultra_right = -500;
    unsigned char SensorReadings::detection_bit = 0x00;
    Planner SensorReadings::planner = planner;
    std::queue<TilePosition> SensorReadings::points_of_interest;
    TilePosition SensorReadings::currentTargetPoint(0,0);
    STATE SensorReadings::current_state = STATE::INIT_SEARCH;
}