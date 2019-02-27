#include "bill_planning/State_Machine.hpp"

StateMachine::StateMachine()
{
    num_states = MachineStates::COUNT;
}

void StateMachine::advanceState(int heading = 0)
{
    ROS_INFO("Advance state, current state: %i", search_state);
    switch (search_state)
    {
        case MachineStates::STARTINGCOURSE:
        {
            // Drive to point X-max Y-0
            // Start initial search (aka drive forward 1 inch scan repeat for course)
            search_state = MachineStates::RUNINITIALSEARCH;
            break;
        }
        case MachineStates::RUNINITIALSEARCH:
        {
            // Start grid search OR drive to saved points from initial search
            search_state = MachineStates::LOOKFORFIRE;
            break;
        }
        case MachineStates::LOOKFORFIRE:
        {
            planner.putOutFire();
            // Start First Half of Scan to Ensure Fire is Out
            planner.PublishTurn((heading + scanning_angle) % 360);

            search_state = MachineStates::SCANNINGFIRE1;
            break;
        }
        case MachineStates::SCANNINGFIRE1:
        {
            planner.PublishTurn((heading + scanning_angle) % 360);
            search_state = MachineStates::SCANNINGFIRE2;
            break;
        }
        case MachineStates::SCANNINGFIRE2:
        {
            planner.PublishTurn((heading - 2 * scanning_angle) % 360);
            //TODO: Update search state with next task
            break;
        }
        default:
        {
            planner.PublishStop();
            break;
        }
    }
}

void StateMachine::resetScanningFire(int heading)
{
    planner.putOutFire();
    search_state = MachineStates::LOOKFORFIRE;
    advanceState(heading);
}