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

// HOW CAN WE WAIT UNTIL WE GET AN ULTRASONIC READING?
// START THREAD AFTER FRONT LEFT AND RIGHT HAVE RECEIVED A READING IN PLANNER MAIN
int main()
{    
    if(!_cleared_fwd && ultra_fwd >= FULL_COURSE_DETECTION_LENGTH)
    {
        _cleared_fwd = true;
    }
    else
    {
        int increment = ultra_left > ultra_right ? -1 : 1;
        desired_heading = ultra_left > ultra_right ? 180 : 0;

        while(!_cleared_fwd)
        {
            int temp_ultra = increment == -1 ? ultra_right : ultra_left;

            if (desired_x_tile == current_x_tile 
                && desired_y_tile == current_y_tile
                && temp_ultra < FULL_COURSE_DETECTION_LENGTH)
            {
                driveToTile(current_x_tile + increment, 0);
                desired_x_tile = current_x_tile + increment;
                desired_y_tile = 0;
            }
            else if (temp_ultra > FULL_COURSE_DETECTION_LENGTH)
            {
                _cleared_fwd = true;
            }
        }
        
        desired_heading = 90;
    }

    driveToTile(current_x_tile, 6);
    desired_x_tile = current_x_tile;
    desired_y_tile = 6;

    while(!_driven_fwd)
    {
        if(desired_x_tile == current_x_tile && desired_y_tile == current_y_tile)
        {
            _driven_fwd = true;
        }
    }

    desired_heading = 0;
    turn(desired_heading);
    while (current_heading != desired_heading);

    _detected_fire = false;
    //delay 0.5 sec if we are facing the fire we want to give some time to detect it

    desired_heading = 180;
    turn(desired_heading);
    while (current_heading != desired_heading)
    {
        if(_detected_fire)
        {
            stop();
            turnFanOn():
            //delay 2 seconds?
            turn(desired_heading);
        }
    }

    
}
