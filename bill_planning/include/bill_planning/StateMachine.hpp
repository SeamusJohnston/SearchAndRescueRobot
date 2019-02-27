#include "bill_planning/Enums.hpp"
#include "bill_planning/Planner.hpp"

struct StateMachine
{
    public:
		StateMachine();
        int search_state;
        void advanceState(int heading);
        void continueAngularScan();
        void publishStop();
        void setMotorPub(ros::Publisher mp);
    private:
        int num_states;
        const int scanning_angle = 10;
        int previous_desired_heading;
		Planner planner;
};
