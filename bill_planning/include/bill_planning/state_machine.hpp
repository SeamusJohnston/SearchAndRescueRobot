#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "bill_planning/enums.hpp"
#include "bill_planning/planner.hpp"
#include "bill_msgs/State.h"

class StateMachine
{
    public:
        StateMachine();
        void advanceState();
        void setupPublishers(ros::Publisher motor_pub, ros::Publisher fan_pub, ros::Publisher state_pub, ros::Publisher led_pub);
        void stateAction();
        void updateHeading(int heading);
        void reset();
        void setFireFlag(bool state);
        void start();
        MajorState major_state;
        MinorState minor_state;
    private:
        const int _scanning_angle = 20;
        Planner _planner;
        bool _found_fire;
        ros::Publisher _state_pub;
        bill_msgs::State _state_msg;
        int _current_heading;
        int _fire_heading;
};

#endif