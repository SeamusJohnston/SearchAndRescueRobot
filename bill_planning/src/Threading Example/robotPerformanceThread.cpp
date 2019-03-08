#include "bill_planning/robotPerformanceThread.hpp"

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

int main()
{
    // WAIT ON DATA FROM EACH ULTRASONIC SENSOR
    while (SensorReadings::start_robot_performance_thread);

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

            if (desired_x_tile == SensorReadings::current_x_tile 
                && desired_y_tile == SensorReadings::current_y_tile
                && temp_ultra < FULL_COURSE_DETECTION_LENGTH)
            {
                SensorReadings::planner.publishDriveToTile(SensorReadings::current_x_tile + increment, 0);
                // THE ULTRASONIC CALLBACK WILL BE IN CHARGE OF SAVING THE POINT OF INTEREST
                desired_x_tile = SensorReadings::current_x_tile + increment;
                desired_y_tile = 0;
            }
            else if (temp_ultra > FULL_COURSE_DETECTION_LENGTH)
            {
                _cleared_fwd = true;
            }
        }
        
        desired_heading = 90;
    }

    SensorReadings::planner.publishDriveToTile(SensorReadings::current_x_tile, 6);
    // THE ULTRASONIC CALLBACK WILL BE IN CHARGE OF SAVING THE POINT OF INTEREST
    desired_x_tile = SensorReadings::current_x_tile;
    desired_y_tile = 6;

    while(!_driven_fwd)
    {
        if(desired_x_tile == SensorReadings::current_x_tile 
            && desired_y_tile == SensorReadings::current_y_tile)
        {
            _driven_fwd = true;
        }
    }

    // Get to desired heading
    desired_heading = 0;
    SensorReadings::planner.publishTurn(desired_heading);
    while (SensorReadings::current_heading != desired_heading);

    desired_heading = 180;
    SensorReadings::planner.publishTurn(desired_heading);
    // THE FIRE CALLBACK WILL BE IN CHARGE OF SAVING THE POINT OF INTEREST

    while (!SensorReadings::points_of_interest.empty())
    {
        //Drive to position
        //While position != desired position {};
        //Determine what the object is, fire, b1, b2
        //if b1 or b2, store location in SensorReadings::determinedPoints[];
        //if fire, call fireOut();
        //if already found fire we can signal we found b1 or b2
    }
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
            SensorReadings::planner.putOutFire();
            desired_heading = current_heading + 2 * FIRE_SCAN_ANGLE;
            temp_desired_heading = current_heading - FIRE_SCAN_ANGLE;

            SensorReadings::planner.publishTurn(temp_desired_heading);

            initialCall = false;
            check_temp_heading = true;
        }
        
        while(check_temp_heading
            && temp_desired_heading != SensorReadings::current_heading)
        {
            if(SensorReadings::detected_fire)
            {
                SensorReadings::planner.putOutFire();
                desired_heading = current_heading + 2 * FIRE_SCAN_ANGLE;
                temp_desired_heading = current_heading - FIRE_SCAN_ANGLE;
                SensorReadings::planner.publishTurn(temp_desired_heading);
            }
        }

        check_temp_heading = false;
        SensorReadings::planner.publishTurn(desired_heading);
    } while(desired_heading != SensorReadings::current_heading);
}