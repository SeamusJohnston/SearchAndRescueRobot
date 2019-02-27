#include "bill_planning/StateMachine.hpp"

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
void fireCallback(const std_msgs::Bool::ConstPtr& msg);
void ultrasonicCallback(const std_msgs::Float32::ConstPtr& msg);

ros::Publisher motor_pub;
ros::Publisher fan_pub;

StateMachine state_machine;

int current_heading = 0;
const int ULTRA_TRIGGER_DIST = 30;
int found_fire = 0;

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // This callback will in the future determine if a goal has been completed, but since the IMU
    // And the encoders aren't operational, it will have to just use timers instead
}

void ultrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // This callback will determine what we should do with ultrasonic data
    // Logic needs to determine what we are searching for or if we need to mark a location
    // Logic needs to determine if we are close enough to complete a given task
}

void fireCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // Only turn fan on if looking/scanning for fire and found fire
    if ((state_machine.search_state == MachineStates::LOOKFORFIRE
        || state_machine.search_state == MachineStates::SCANNINGFIRE
        || state_machine.search_state == MachineStates::AWAITINGFIRESCAN)
        && msg->data)
    {
        // Multiple hits in case of noise or bumps (causing sensor to point at light)
        // TODO: This may not work for scanning depending how fast we scan... might have
        //       only detect once while doing a scan
        if (found_fire < 3)
        {
            found_fire++;
        }
        else
        {
            state_machine.publishStop()

            std_msgs::Bool cmd;
            cmd.data = true;

            fan_pub.publish(cmd); // Turn on fan
            ros::Duration(2).sleep();

            cmd.data = false;
            fan_pub.publish(cmd); // Turn off fan

            if (state_machine.search_state == MachineStates::LOOKFORFIRE)
            {
                state_machine.advanceState(current_heading);
            }
            else
            {
                state_machine.continueAngularScan();
            }
            
        }
    }
    else
    {
        found_fire = 0;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "game_day_planner");
    ros::NodeHandle nh;

    // Subscribing to Topics
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 1, fusedOdometryCallback);
    ros::Subscriber sub_fire = nh.subscribe("fire", 1, fireCallback);
    ros::Subscriber sub_ultrasonic = nh.subscribe("ultrasonic", 1, ultrasonicCallback);
    
    motor_pub = nh.advertise<bill_msgs::MotorCommands>("motor_cmd", 100);
    fan_pub = nh.advertise<std_msgs::Bool>("fan", 100);

    // Start state machine, then spin
    state_machine.setMotorPub(motor_pub);
    state_machine.search_state = MachineStates::STARTINGCOURSE;
    state_machine.advanceState(current_heading);

    ros::spin();
    return 0;
}
