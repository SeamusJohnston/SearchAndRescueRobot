#ifndef SENSOR_READINGS_HPP
#define SENSOR_READINGS_HPP

#include <queue>
#include <mutex>
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
        SensorReadings();
        void setStartRobotPerformanceThread(bool val);
        bool getStartRobotPerformanceThread();

        void setUltraFwd(float val);
        float getUltraFwd();
        void setUltraLeft(float val);
        float getUltraLeft();
        void setUltraRight(float val);
        float getUltraRight();

        // WHEN SET TO TRUE, THE THREAD WILL START
        void setDetectedFire(bool val);
        bool getDetectedFire();

        void setCurrentHeading(int val);
        int getCurrentHeading();

        void setCurrentTileX(int val);
        void setCurrentTileY(int val);
        int getCurrentTileX();
        int getCurrentTileY();

        void setCurrentState(STATE val);
        STATE getCurrentState();

        void setDetectionBit(unsigned char val);
        unsigned char getDetectionBit();

        std::queue<TilePosition> points_of_interest;
        TilePosition currentTargetPoint;

        //Set to 1 if fire, 2 if small building and 3 if large
        unsigned char detection_bit;

    private:
        std::mutex _start_robot_performance_thread_mutex;
        std::mutex _detected_fire_mutex;        
        bool _start_robot_performance_thread;
        bool _detected_fire;

        std::mutex _ultra_fwd_mutex;
        std::mutex _ultra_left_mutex;
        std::mutex _ultra_right_mutex;
        float _ultra_fwd;
        float _ultra_left;
        float _ultra_right;

        std::mutex _current_heading_mutex;
        int _current_heading;

        std::mutex _current_tile_mutex;
        TilePosition _current_tile;

        std::mutex _current_state_mutex;
        STATE _current_state;

        std::mutex _detection_bit_mutex;
        unsigned char _detection_bit;
};

#endif