#include "bill_planning/sensorReadings.hpp"

SensorReadings::SensorReadings(Planner p)
{
    current_x_tile = 0;
    current_y_tile = 0;
    current_heading = 90;
    detected_fire = false;
    start_robot_performance_thread = false;

    planner = p;
}