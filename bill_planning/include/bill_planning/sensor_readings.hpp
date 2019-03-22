#ifndef SENSOR_READINGS_HPP
#define SENSOR_READINGS_HPP

#include <queue>
#include <mutex>
#include <thread>
#include <list>
#include <algorithm>
#include "position.hpp"
#include <set>
#include <utility>  
#include <vector>
#include "ros/ros.h"

enum STATE
{
    FINDING_T_SEARCH_TILE = 0,
    INIT_SEARCH = 1,
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

        void setDetectedFireFwd(bool val);
        void setDetectedFireLeft(bool val);
        void setDetectedFireRight(bool val);
        bool getDetectedFireFwd();
        bool getDetectedFireLeft();
        bool getDetectedFireRight();

        void setCurrentHeading(int val);
        int getCurrentHeading();

        void setTargetHeading(int val);
        int getTargetHeading();

        void setCurrentTileX(int val);
        void setCurrentTileY(int val);
        int getCurrentTileX();
        int getCurrentTileY();

        void setCurrentPositionX(float val);
        void setCurrentPositionY(float val);
        float getCurrentPositionX();
        float getCurrentPositionY();

        void setTargetPoint(int x, int y);
        int getTargetTileX();
        int getTargetTileY();
        bool isTargetTileValid();
        void invalidateTargetTile();

        void setCurrentState(STATE val);
        STATE getCurrentState();

        void setDetectionBit(unsigned char val);
        unsigned char getDetectionBit();

        bool pointsOfInterestEmpty();
        int pointsOfInterestSize();
        TilePosition pointsOfInterestFront();
        void pointsOfInterestPop();
        void pointsOfInterestEmplace(TilePosition tp);

        int freeRowTile();
        void updateFlameTileFromLastSavedPoint(bool is_left);
        
        void setFlameTileX(int val);
        void setFlameTileY(int val);
        int getFlameTileX();
        int getFlameTileY();

        void setHomeTile(int x, int y);
        int getHomeTileX();
        int getHomeTileY();

        //Set to 1 if fire, 2 if small building and 3 if large
        unsigned char detection_bit;

    private:
        std::mutex _target_tile_mutex;
        std::mutex _current_tile_mutex;
        std::mutex _current_position_mutex;
        std::mutex _flame_tile_mutex;
        std::mutex _home_tile_mutex;
        std::mutex _points_of_interest_mutex;
        std::mutex _start_robot_performance_thread_mutex;
        std::mutex _current_state_mutex;
        std::mutex _ultra_fwd_mutex;
        std::mutex _ultra_left_mutex;
        std::mutex _ultra_right_mutex;
        std::mutex _current_heading_mutex;
        std::mutex _target_heading_mutex;
        std::mutex _detection_bit_mutex;
        std::mutex _detected_fire_mutex;

        // Stored y,x for future use
        std::vector<int> _y_Objects = {0,1,2,3,4,5};
        std::vector<std::pair <int, int>> _s = {};
        std::queue<TilePosition> _points_of_interest;
        STATE _current_state = INIT_SEARCH;


        bool _start_robot_performance_thread = false;
        bool _detected_fire_fwd = false;
        bool _detected_fire_left = false;
        bool _detected_fire_right = false;

        float _ultra_fwd = -500;
        float _ultra_left = -500;
        float _ultra_right = -500;
        int _current_heading = 90;
        int _target_heading = 90;
        unsigned char _detection_bit = 0x00;

        Position _current_position = Position(0,0);
        TilePosition _home_tile = TilePosition(-1,-1);
        TilePosition _flame_tile = TilePosition(-1, -1);
        TilePosition _current_tile = TilePosition(-1, -1);
        TilePosition _current_target_tile = TilePosition(-1, -1);
};

#endif
