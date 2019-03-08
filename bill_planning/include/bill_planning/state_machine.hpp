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
    void setupPublishers(ros::Publisher motor_pub, ros::Publisher fan_pub, ros::Publisher state_pub,
                         ros::Publisher led_pub);
    void stateAction();
    void updateHeading(int heading);
    void reset();
    void setFireFlag(bool state);
    void updatePosition(Position pos);
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
    int _current_tile_x;
    int _current_tile_y;
    int _desired_tile_x;
    int _desired_tile_y;

    // We need a queue or list of saved points?

    bool _finish_straight_search;
    bool _finish_L_search;
    bool _finish_T_search;
};

#endif