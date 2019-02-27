#include "ros/ros.h"
#include "bill_msgs/MotorCommands.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <mutex>

#define ULTRA_TRIGGER_DIST 30

// Forward declarations
void publishStop();
void publishDrive(const int heading, const float speed);
void publishTurn(const int heading);

enum machineStatus
{
    STOPPED = 0,
    RUNNING = 1
};

struct StateMachine
{
    const static int num_states = 4;
    machineStatus status = machineStatus::STOPPED;
    static int state;
    ros::Timer timer;
    static void timerCallback(const ros::TimerEvent&)
    {
        advanceState();
    }
    static void advanceState()
    {
        switch (state)
        {
            ROS_INFO("Advance state, current state: %i", state);
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
        ROS_INFO("State machine running.");
        status = machineStatus::RUNNING;
        timer.start();
    }

    void pauseMachine()
    {
        ROS_INFO("State machine stopped.");
        status = machineStatus::STOPPED;
        timer.stop();
    }
};

ros::Publisher motor_pub;
ros::Publisher fan_pub;
bill_msgs::MotorCommands command_msg;
StateMachine state_machine;
int StateMachine::state = 0;
// std::mutex mutex;
int current_heading = 0;

void publishStop()
{
    // std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding stop");
    command_msg.command = bill_msgs::MotorCommands::STOP;
    command_msg.heading = 0;
    command_msg.speed = 0;
    motor_pub.publish(command_msg);
}

void publishDrive(const int heading, const float speed)
{
    // std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding drive, heading: %i and speed: %f", heading, speed);
    command_msg.command = bill_msgs::MotorCommands::DRIVE;
    command_msg.heading = heading;
    command_msg.speed = speed;
    motor_pub.publish(command_msg);
}

void publishTurn(const int heading)
{
    // std::lock_guard<std::mutex> lk(mutex);
    ROS_INFO("Commanding turn, heading: %i", heading);
    command_msg.command = bill_msgs::MotorCommands::TURN;
    command_msg.heading = heading;
    command_msg.speed = 0;  // Speed is hardcoded in the motor driver for turning
    motor_pub.publish(command_msg);
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
        // std::lock_guard<std::mutex> lk(mutex);
        state_machine.pauseMachine();
        publishStop();

        std_msgs::Bool cmd;
        cmd.data = true;
        fan_pub.publish(cmd);  // Turn on fan
        ros::Duration(2).sleep();

        cmd.data = false;
        fan_pub.publish(cmd);  // Turn off fan
        state_machine.runMachine();
    }
}

void ultrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if (msg->data <= ULTRA_TRIGGER_DIST)
    {
        // std::lock_guard<std::mutex> lk(mutex);
        state_machine.pauseMachine();
        publishTurn((current_heading + 10) % 360);  // Request rotation by 10 degrees clockwise
        // Since IMU isn't operational, the heading published does nothing but set the direction
        // Therefore we have to start a short timer to actually represent the 10 degree turn
        ros::Duration(0.5).sleep();
        // TODO: Current heading should update from IMU once working, so remove this when it does
        current_heading += 10;
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
    ros::Subscriber sub_ultrasonic = nh.subscribe("ultrasonic", 1, ultrasonicCallback);
    motor_pub = nh.advertise<bill_msgs::MotorCommands>("motor_cmd", 100);
    fan_pub = nh.advertise<std_msgs::Bool>("fan", 100);

    // Start state machine, then spin
    state_machine.status = machineStatus::RUNNING;
    state_machine.timer = nh.createTimer(ros::Duration(5), StateMachine::timerCallback);
    state_machine.advanceState();
    ros::spin();
    return 0;
}
