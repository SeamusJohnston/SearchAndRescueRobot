#ifndef STATIC_VARIABLES_HPP
#define STATIC_VARIABLES_HPP

#include <queue>
#include "bill_planning/planner.hpp"

struct sensorReadings
{
    public:
        sensorReadings(Planner p);
        static int current_x_tile;
        static int current_y_tile;

        static int current_heading;

        static float ultra_fwd;
        static float ultra_left;
        static float ultra_right;

        static bool detected_fire;

        Planner planner;
        static std::queue<Position> pointsOfInterest;
};

#endif