#include "bill_planning/Enums.hpp"
#include "bill_planning/Planner.hpp"

struct StateMachine
{
    public:
        StateMachine();
        int search_state;
        void advanceState(int heading = 0);
        void resetScanningFire(int heading):
    private:
        int num_states;
        const int scanning_angle = 10;
        Planner planner;
};
