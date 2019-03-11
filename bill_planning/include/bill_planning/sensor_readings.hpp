#ifndef SENSOR_READINGS_HPP
#define SENSOR_READINGS_HPP

#include <queue>
#include "bill_planning/planner.hpp"


enum STATE
{
    INIT_SEARCH = 0,
    INIT_FIRE_SCAN = 1,
    FLAME_SEARCH = 2,
    BUILDING_SEARCH = 3,
    HALL_SEARCH = 4,
    RETURN_HOME = 5
};

class SensorReadings
{
    public:
        // WHEN SET TO TRUE, THE THREAD WILL START
        static bool start_robot_performance_thread;

        static float ultra_fwd;
        static float ultra_left;
        static float ultra_right;
        static bool detected_fire;

        static int current_heading;
        static Planner * planner;
        static std::queue<TilePosition> points_of_interest;
        static TilePosition current_tile;
        static TilePosition currentTargetPoint;

        static STATE current_state;

        //Set to 1 if fire, 2 if small building and 3 if large
        static unsigned char detection_bit;
};

#endif