#include "bill_planner/State_Machine.hpp"

State_Machine::State_Machine(Planner p)
{
    planner = p;
}

State_Machine::advanceState(int heading = 0)
{
    switch (state
    {
        ROS_INFO("Advance state, current state: %i", state);
        case MachineStates:LOOKFORFIRE
        {
            search_state == MachineStates::SCANNINGFIRE;
            planner.scanFire(heading + scanning_angle)
            break;
        }
        case MachineStates:SCANNINGFIRE
        {
            search_state == MachineStates::AWAITINGFIRESCAN;
            planner.scanFire(heading - 2 * scanning_angle)
            break;
        }
        case MachineStates::AWAITINGFIRESCAN:
        {
            //Found and Put out the fire, what's next?
        }
        default:
        {
            publishStop();
            break;
        }
    }
}