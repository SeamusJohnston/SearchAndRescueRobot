#include "ros/ros.h"
#include "bill_msgs/MotorCommands.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <mutex>

#define ULTRA_TRIGGER_DIST 30

enum machineStatus{
    STOPPED = 0,
    RUNNING = 1
};

struct StateMachine{
    const int num_states = 4;
    machineStatus status = machineStatus::STOPPED;
    int state = 0;
    ros::Timer timer;
    void timerCallback(const ros::TimerEvent&)
    {
        advanceState();
    }
    void advanceState()
    {
        switch (state)
        {
            case 0:
            {
                publishStop();
                break;
            }
            case 1:
            {
                publishDrive(0, 0.1);
                break;
            }
            case 2:
            {
                publishStop();
                break;
            }
            case 3:
            {
                publishDrive(0, -0.1);
                break;
            }
            default:
            {
                publishStop();
                break;
            }
        }
        state = (state + 1) % num_states;
    }

    void runMachine()
    {
        status = machineStatus::RUNNING;
        timer.start();
    }

    void pauseMachine()
    {
        status = machineStatus::STOPPED;
        timer.stop();
    }
};

ros::Publisher motor_pub;
ros::Publisher fan_pub;
bill_msgs::MotorCommands *msg = NULL;
StateMachine state_machine;
std::mutex mutex;

void publishStop()
{
    std::lock_guard<std::mutex> lk(mutex);
    msg->command = bill_msgs::MotorCommands::STOP;
    msg->heading = 0;
    msg->speed = 0;
    motor_pub.publish(&msg);
}

void publishDrive(const int heading, const float speed)
{
    std::lock_guard<std::mutex> lk(mutex);
    msg->command = bill_msgs::MotorCommands::DRIVE;
    msg->heading = heading;
    msg->speed = speed;
    motor_pub.publish(&msg);
}

void publishTurn(const int heading)
{
    std::lock_guard<std::mutex> lk(mutex);
    msg->command = bill_msgs::MotorCommands::TURN;
    msg->heading = heading;
    msg->speed = 0; // Speed is hardcoded in the motor driver for turning
    motor_pub.publish(&msg);
}

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // This callback will in the future determine if a goal has been completed, but since the IMU
    // And the encoders aren't operational, it will have to just use timers instead
}

void fireCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        std::lock_guard<std::mutex> lk(mutex);
        state_machine.pauseMachine();
        publishStop();

        std_msgs::Bool cmd;
        cmd.data = true;
        fan_pub.publish(cmd); // Turn on fan
        ros::Duration(2).sleep();

        cmd.data = false;
        fan_pub.publish(cmd); // Turn off fan
        state_machine.runMachine();
    }
}

void ultrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if (msg->data <= ULTRA_TRIGGER_DIST)
    {
        std::lock_guard<std::mutex> lk(mutex);
        state_machine.pauseMachine();
        publishTurn((current_heading + 10) % 360); // Request rotation by 10 degrees clockwise
        // Since IMU isn't operational, the heading published does nothing but set the direction
        // Therefore we have to start a short timer to actually represent the 10 degree turn
        ros::Duration(0.5).sleep();

        publishStop();
        state_machine.runMachine();
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_planner");
    ros::NodeHandle nh;
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 1, fusedOdometryCallback);
    ros::Subscriber sub_fire = nh.subscribe("fire", 1, fireCallback);
    ros::Subscriber sub_ultrasonic = nh.subscribe("ultrasonic", 10, ultrasonicCallback);
    motor_pub = nh.advertise<bill_msgs::MotorCommands>("motor_cmd", 100);
    fan_pub = nh.advertise<std_msgs::Bool>("fan", 100);

    // Start state machine, then spin
    state_machine.status = machineStatus::RUNNING;
    state_machine.timer = nh.createTimer(ros::Duration(5), state_machine::timerCallback);
    state_machine.advanceState();
    ros::spin();
    return 0;
}
