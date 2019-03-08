#ifndef STATIC_VARIABLES_HPP
#define STATIC_VARIABLES_HPP


static struct staticVariables
{
    public:
        static int current_x_tile = 0;
        static int current_y_tile = 0;

        static int current_heading = 90;

        static float ultra_fwd;
        static float ultra_left;
        static float ultra_right;

        static bool _detected_fire = false;

        std::queue pointsOfInterest;
}

#endif