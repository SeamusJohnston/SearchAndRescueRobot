#include "bill_planning/State_Machine.hpp"

State_Machine::State_Machine()
{
    previousDesiredHeading = 0;
    num_states = MachineStates::COUNT;
}

void State_Machine::AdvanceState(int heading = 0)
{
    switch (search_state)
    {
        ROS_INFO("Advance state, current state: %i", search_state);
        case MachineStates::LOOKFORFIRE:
        {
            // Start First Half of Scan to Ensure Fire is Out
            previousDesiredHeading = (heading + scanning_angle) % 360;
            planner.PublishTurn(previousDesiredHeading);

            search_state = MachineStates::SCANNINGFIRE;
            break;
        }
        case MachineStates::SCANNINGFIRE:
        {
            // Conduct Second Half of Scan to Ensure Fire is Out
            previousDesiredHeading = (heading - 2 * scanning_angle) % 360;
            planner.PublishTurn(previousDesiredHeading);
            search_state = MachineStates::AWAITINGFIRESCAN;
            break;
        }
        case MachineStates::AWAITINGFIRESCAN:
        {
            //Found and Put out the fire, what's next?
            break;
        }
        default:
        {
            planner.PublishStop();
            break;
        }
    }
}

void State_Machine::ContinueAngularScan()
{
    planner.ScanAngle(previousDesiredHeading);
}
