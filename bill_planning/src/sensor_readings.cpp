#include "bill_planning/sensor_readings.hpp"

SensorReadings::SensorReadings()
{
    current_x_tile = 0;
    current_y_tile = 0;
    current_heading = 90;
    detected_fire = false;
    start_robot_performance_thread = false;
}

void SensorReadings::setPlanner(Planner p)
{
    planner = p;
}