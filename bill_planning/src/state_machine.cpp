#include "bill_planning/state_machine.hpp"

StateMachine::StateMachine()
{
    _found_fire = false;
    _hit_corner = false;
    _current_heading = 0;
    _fire_heading = 0;
}

void StateMachine::setupPublishers(const ros::Publisher motor_pub, const ros::Publisher fan_pub,
                                   const ros::Publisher state_pub, const ros::Publisher led_pub)
{
    _planner.setPubs(motor_pub, fan_pub, led_pub);
    _state_pub = state_pub;
}

void StateMachine::updateHeading(const int heading)
{
    _current_heading = heading;

    // If verifying fire, this callback is used for changing direction of scan
    if ((major_state == SEARCH_FIRE || major_state == INIT_SEARCH) && minor_state == VERIFY_FIRE)
    {
        if (_current_heading >= (_fire_heading + _scanning_angle) % 360)
        {
            // Hit first heading, now turn the other way
            int new_heading = ((_fire_heading - 2 * _scanning_angle) + 360) % 360;
            _planner.publishTurn(new_heading);
        }
        if (_current_heading <= ((_fire_heading - 2 * _scanning_angle) + 360) % 360)
        {
            // End of scan, we may advance
            advanceState();
        }
    }

    // TODO BM If doing init search and heading is updated, drive forward to corner
}

void StateMachine::updatePosition(Position pos)
{
    
}

void StateMachine::advanceState()
{
    if (major_state == SEARCH_FIRE)
    {
        if (minor_state == RUN)
        {
            // If advancing, we must have found fire
            minor_state = FOUND_FIRE;
        }
        else if (minor_state == FOUND_FIRE)
        {
            // If advancing, we think the fire has been put out so verify that that is true
            minor_state = VERIFY_FIRE;
        }
        else if (minor_state == VERIFY_FIRE && _found_fire)
        {
            // If advancing and fire is still on, we found the fire while verifying so reverse the state machine
            minor_state = FOUND_FIRE;
            // TODO: If this is our second pass maybe we need to move closer, could either happen here, or we set a
            // flag?
        }
        else
        {
            // If advancing and fire was not seen again, then we have completed the state
            minor_state = COMPLETE;
        }
    }
    else if (major_state == INIT_SEARCH)
    {
        if (minor_state == RUN && _found_fire)
        {
            minor_state = FOUND_FIRE;
        }
        else if (minor_state == FOUND_FIRE)
        {
            minor_state = VERIFY_FIRE;
        }
        else if (minor_state == VERIFY_FIRE && _found_fire)
        {
            minor_state = FOUND_FIRE;
        }
        else
        {
            // If advancing and fire was not seen again, then we have completed the state
            minor_state = COMPLETE;
        }
    }
    // TODO: Implement rest of major states

    // Publish current state for debug purposes
    _state_msg.major_state = major_state;
    _state_msg.minor_state = minor_state;

    _state_pub.publish(_state_msg);

    stateAction();
}

void StateMachine::stateAction()
{
    if (major_state == MajorState::SEARCH_FIRE)
    {
        if (minor_state == RUN)
        {
            // Perform search trying to find fire
            _planner.gridSearch();
        }
        else if (minor_state == FOUND_FIRE)
        {
            // Have found fire, so put it out
            _planner.putOutFire();
            // Assume we put it out unless told otherwise
            _found_fire = false;
            _fire_heading = _current_heading;
            // We may now advance, since this is the only action for this state
            advanceState();  // TODO: maybe give this more thought these two functions are calling each other, should be
                             // fine though
        }
        else if (minor_state == VERIFY_FIRE)
        {
            // Need to verify that the fire is out, so start by scanning CW
            _planner.publishTurn((_fire_heading + _scanning_angle) % 360);
        }
        else
        {
            // Turn on led to indicate we have successfully put out the fire
            _planner.signalComplete();
            // TODO: advance state here?
        }
    }
}

void StateMachine::reset()
{
    _planner.publishStop();
    _found_fire = false;
    _hit_corner = false;
    _fire_heading = 0;
}

void StateMachine::setFireFlag(const bool state)
{
    _found_fire = state;
}

void StateMachine::setCornerFlag(const bool state)
{
    _hit_corner = state;
}

bool StateMachine::getCornerFlag()
{
    return _hit_corner;
}

void StateMachine::start()
{
    major_state = INIT_SEARCH;
    // TODO BM SET DESIRED HEADING TO LEFT
    minor_state = RUN;
    stateAction();
}