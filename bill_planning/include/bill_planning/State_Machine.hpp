#include "ros/ros.h"
#include "bill_msgs/MotorCommands.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include "bill_planner/Enums.hpp"
#include "bill_planner/Planner.hpp"

struct State_Machine
{
    public:
        static int search_state;
        static Planner planner;

        State_Machine();
        static void advanceState();
        void scanFire();
    private:
        const static int num_states = MachineStates::COUNT;
        const int scanning_angle = 10;
}