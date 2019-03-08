#include "bill_planning/sensorReadings.hpp"

sensorReadings::sensorReadings(Planner p)
{
    current_x_tile = 0;
    current_y_tile = 0;
    current_heading = 90;
    detected_fire = false;
    
    planner = p;
    // pointsOfInterest = new std::queue<Position>;
}