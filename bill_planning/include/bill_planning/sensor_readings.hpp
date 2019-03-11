#ifndef SENSOR_READINGS_HPP
#define SENSOR_READINGS_HPP

#include <queue>
#include "bill_planning/planner.hpp"


enum STATE
{
    INIT_SEARCH = 0,
    FLAME_SEARCH = 1,
    BUILDING_SEARCH = 2,
    HALL_SEARCH = 3,
    RETURN_HOME = 4
};

class SensorReadings
{
    public:
        SensorReadings();

        static int current_heading;

        static float ultra_fwd;
        static float ultra_left;
        static float ultra_right;

        // WHEN SET TO TRUE, THE THREAD WILL START
        // ONLY SET WHEN WE HAVE RECEIVED FRONT, LEFT AND RIGHT ULTRASONIC READINGS
        static bool start_robot_performance_thread;
        static bool detected_fire;


        static Planner planner;
        static std::queue<TilePosition> points_of_interest;
        static TilePosition current_tile;
        static TilePosition currentTargetPoint;

        static STATE current_state;

        //Set to 1 if fire, 2 if small building and 3 if large
        static unsigned char detection_bit;
};

#endif