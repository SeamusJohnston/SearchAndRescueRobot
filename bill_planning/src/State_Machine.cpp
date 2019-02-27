#include "bill_planning/State_Machine.hpp"

StateMachine::StateMachine()
{
    previous_desired_heading = 0;
    num_states = MachineStates::COUNT;
}

void StateMachine::setMotorPub(ros::Publisher mp)
{
    planner.setMotorPub(mp);
}

void StateMachine::advanceState(int heading = 0)
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

void StateMachine::continueAngularScan()
{
    planner.ScanAngle(previousDesiredHeading);
}

void StateMachine::publishStop()
{
    planner.publishStop();
}
