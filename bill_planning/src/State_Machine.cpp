#include "bill_planning/Planner.hpp"

State_Machine::State_Machine()
{
}

State_Machine::AdvanceState(int heading = 0)
{
    switch (state)
    {
        ROS_INFO("Advance state, current state: %i", state);
        case MachineStates:LOOKFORFIRE
        {
            // Start First Half of Scan to Ensure Fire is Out
            previousDesiredHeading = heading + scanning_angle;
            planner.ScanAngle(previousDesiredHeading)

            search_state == MachineStates::SCANNINGFIRE;
            break;
        }
        case MachineStates:SCANNINGFIRE
        {
            // Conduct Second Half of Scan to Ensure Fire is Out
            previousDesiredHeading = heading - 2 * scanning_angle;
            planner.ScanAngle(previousDesiredHeading)
            search_state == MachineStates::AWAITINGFIRESCAN;
            break;
        }
        case MachineStates::AWAITINGFIRESCAN:
        {
            //Found and Put out the fire, what's next?
        }
        default:
        {
            planner.PublishStop();
            break;
        }
    }
}

State_Machine::ContinueAngularScan()
{
    planner.ScanAngle(previousDesiredHeading)
}