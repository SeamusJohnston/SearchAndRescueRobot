#include "bill_planning/Enums.hpp"
#include "bill_planning/Planner.hpp"

struct State_Machine
{
    public:
	State_Machine();
        int search_state;
        Planner planner;

        void AdvanceState(int heading);
        void ContinueAngularScan();
    private:
        static int num_states;
        const int scanning_angle = 10;
        int previousDesiredHeading;
};
