#include "bill_planning/robot_performance_thread.hpp"

//DIMENSIONS IN METRES. WILL NEED A STATIC STRUCT OR CLASS FOR SENSOR VALUES
//MUST EXTRACT THE CLASS

// FUNCTIONS
void fireOut();
void findClearPathFwd();
void completeStraightLineSearch();
void scanForFire();
void driveToDesiredPoints();
//TODO
void conductGridSearch();
void driveToLargeBuilding();

// FLAGS
bool _cleared_fwd = false;
bool _driven_fwd = false;

// POSITION
TilePosition desired_tile(0,0);
TilePosition large_building_tile(0,0);
int desired_heading = 90;

// CONSTANTS
const int FULL_COURSE_DETECTION_LENGTH = 1.70;
const int FIRE_SCAN_ANGLE = 20;

// SensorReadings Initialization
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

int main()
{
    // WAIT ON DATA FROM EACH ULTRASONIC SENSOR
    while (SensorReadings::start_robot_performance_thread);

    findClearPathFwd();

    completeStraightLineSearch();
    SensorReadings::current_state = STATE::INIT_FIRE_SCAN;

    scanForFire();

    if(SensorReadings::points_of_interest.size() < 3)
    {
        // SOMETHING WENT WRONG WE SHOULD HAVE DETECTED BY NOW
        // THIS PROBABLY MEANS THEY ARE BACK TO BACK
    }

    SensorReadings::current_state = STATE::FLAME_SEARCH;
    driveToDesiredPoints();
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
            desired_heading = SensorReadings::current_heading + 2 * FIRE_SCAN_ANGLE;
            temp_desired_heading = SensorReadings::current_heading - FIRE_SCAN_ANGLE;

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
                desired_heading = SensorReadings::current_heading + 2 * FIRE_SCAN_ANGLE;
                temp_desired_heading = SensorReadings::current_heading - FIRE_SCAN_ANGLE;
                SensorReadings::planner.publishTurn(temp_desired_heading);
            }
        }

        check_temp_heading = false;
        SensorReadings::planner.publishTurn(desired_heading);
    } while(desired_heading != SensorReadings::current_heading);

    SensorReadings::current_state = STATE::BUILDING_SEARCH;
}

void findClearPathFwd()
{
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

            if (desired_tile.x == SensorReadings::current_tile.x 
                && desired_tile.y == SensorReadings::current_tile.y
                && temp_ultra < FULL_COURSE_DETECTION_LENGTH)
            {
                SensorReadings::planner.publishDriveToTile(
                        SensorReadings::current_tile.x,
                        SensorReadings::current_tile.y,
                        SensorReadings::current_tile.x + increment,
                        0, 0.4);
                // THE ULTRASONIC CALLBACK WILL BE IN CHARGE OF SAVING THE POINT OF INTEREST
                desired_tile.x = SensorReadings::current_tile.x + increment;
                desired_tile.y = 0;
            }
            else if (temp_ultra > FULL_COURSE_DETECTION_LENGTH)
            {
                _cleared_fwd = true;
            }
        }
        
        desired_heading = 90;
        SensorReadings::planner.publishTurn(desired_heading);
    }
}

void completeStraightLineSearch()
{
    desired_tile.x = SensorReadings::current_tile.x;
    desired_tile.y = 6;

    SensorReadings::planner.publishDriveToTile(SensorReadings::current_tile.x,
                                            SensorReadings::current_tile.y,
                                            desired_tile.x,
                                            desired_tile.y, 0.4);

    while(!_driven_fwd)
    {
        if(desired_tile.x == SensorReadings::current_tile.x 
            && desired_tile.y == SensorReadings::current_tile.y)
        {
            _driven_fwd = true;
        }
    }
}

void scanForFire()
{
    desired_heading = 0;
    SensorReadings::planner.publishTurn(desired_heading);
    while (SensorReadings::current_heading != desired_heading);

    // 181 ENSURES FRONT WILL DO THE SCANNING (WE ARE AT TOP OF GRID)
    // AND THE ULTRASONIC CAN AID IN OBJECT DETECTION
    desired_heading = 181;
    SensorReadings::planner.publishTurn(desired_heading);
    while (SensorReadings::current_heading != desired_heading);
}

void driveToDesiredPoints()
{
    while (!SensorReadings::points_of_interest.empty())
    {
        SensorReadings::currentTargetPoint = SensorReadings::points_of_interest.front();
        SensorReadings::points_of_interest.pop();

        desired_tile.x = SensorReadings::currentTargetPoint.x;
        desired_tile.y = SensorReadings::currentTargetPoint.y;

        SensorReadings::planner.publishDriveToTile(SensorReadings::current_tile.x,
                                                   SensorReadings::current_tile.y,
                                                   desired_tile.x,
                                                   desired_tile.y, 0.4);

        //Drive to position
        while (SensorReadings::current_tile.x != desired_tile.x
            && SensorReadings::current_tile.y != desired_tile.y);

        // TODO BEFORE WHILE 
        // We could possibly trigger a scan in here but maybe we should create a scan tile
        // function in planner to search tile for one of the 3 objects
        while (SensorReadings::detection_bit == 0x00)
        {
        }

        if (SensorReadings::detection_bit == 0x01)
        {
            fireOut();
        }
        else if (SensorReadings::current_state = STATE::BUILDING_SEARCH)
        {
            SensorReadings::planner.signalComplete();
            if (SensorReadings::detection_bit == 0x03)
            {
                large_building_tile.x = SensorReadings::current_tile.x;
                large_building_tile.x = SensorReadings::current_tile.y;
            }
        }
        else
        {
            SensorReadings::points_of_interest.push(SensorReadings::currentTargetPoint);
        }
    }
}


void conductGridSearch()
{

}

void driveToLargeBuilding()
{
    SensorReadings::currentTargetPoint.x = large_building_tile.x;
    SensorReadings::currentTargetPoint.y = large_building_tile.y;

    desired_tile.x = large_building_tile.x;
    desired_tile.y = large_building_tile.y;

    
    SensorReadings::planner.publishDriveToTile(SensorReadings::current_tile.x,
                                            SensorReadings::current_tile.y,
                                            desired_tile.x,
                                            desired_tile.y, 0.4);
    
    while (SensorReadings::current_tile.x != desired_tile.x
        && SensorReadings::current_tile.y != desired_tile.y);

    // TODO
    // We could possibly trigger a scan in here but maybe we should create a scan tile
    // function in planner to search tile for one of the 3 objects
    while (SensorReadings::detection_bit == 0x00)
    {
    }
}