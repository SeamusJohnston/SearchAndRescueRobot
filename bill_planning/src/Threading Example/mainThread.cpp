#include "bill_planning/sensorReadings.hpp"

//DIMENSIONS IN METRES. WILL NEED A STATIC STRUCT OR CLASS FOR SENSOR VALUES
//MUST EXTRACT THE CLASS

// FLAGS
bool _cleared_fwd = false;
bool _driven_fwd = false;

// POSITION
int desired_x_tile = 0;
int desired_y_tile = 0;

int desired_heading = 90; 
const int FULL_COURSE_DETECTION_LENGTH = 1.70;
const int FIRE_SCAN_ANGLE = 20;

// HOW CAN WE WAIT UNTIL WE GET AN ULTRASONIC READING?
// START THREAD AFTER FRONT LEFT AND RIGHT HAVE RECEIVED A READING IN PLANNER MAIN
int main()
{
    if(!_cleared_fwd && sensorReadings::ultra_fwd >= FULL_COURSE_DETECTION_LENGTH)
    {
        _cleared_fwd = true;
    }
    else
    {
        int increment = sensorReadings::ultra_left > sensorReadings::ultra_right ? -1 : 1;
        desired_heading = sensorReadings::ultra_left > sensorReadings::ultra_right ? 180 : 0;

        while(!_cleared_fwd)
        {
            int temp_ultra = increment == -1 ? 
                sensorReadings::ultra_right : 
                sensorReadings::ultra_left;

            if (desired_x_tile == sensorReadings::current_x_tile 
                && desired_y_tile == sensorReadings::current_y_tile
                && temp_ultra < FULL_COURSE_DETECTION_LENGTH)
            {
                //driveToTile(sensorReadings::current_x_tile + increment, 0);
                desired_x_tile = sensorReadings::current_x_tile + increment;
                desired_y_tile = 0;
            }
            else if (temp_ultra > FULL_COURSE_DETECTION_LENGTH)
            {
                _cleared_fwd = true;
            }
        }
        
        desired_heading = 90;
    }

    //driveToTile(sensorReadings::current_x_tile, 6);
    desired_x_tile = sensorReadings::current_x_tile;
    desired_y_tile = 6;

    while(!_driven_fwd)
    {
        if(desired_x_tile == sensorReadings::current_x_tile 
            && desired_y_tile == sensorReadings::current_y_tile)
        {
            _driven_fwd = true;
        }
    }

    // Get to desired heading
    desired_heading = 0;
    sensorReadings::planner.publishTurn(desired_heading);
    while (sensorReadings::current_heading != desired_heading);

    //TODO BM: DRIVE TO FIRE
    // then call fireOut();
}

void fireOut()
{
    sensorReadings::planner.putOutFire();
    int temp_desired_heading = current_heading - FIRE_SCAN_ANGLE;
    sensorReadings::planner.publishTurn(temp_desired_heading);

    while(temp_desired_heading != sensorReadings::current_heading)
    {
        if(sensorReadings::detected_fire)
        {
            sensorReadings::planner.putOutFire();
            temp_desired_heading = current_heading - FIRE_SCAN_ANGLE;
            sensorReadings::planner.publishTurn(temp_desired_heading);
        }
    }
}