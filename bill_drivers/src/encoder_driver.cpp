#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "wiringPi.h"
#include "angles/angles.h"
#include "bill_drivers/constant_definition.hpp"
#include "bill_msgs/MotorCommands.h"
#include <cmath>

// TODO: Get an accurate measurement of the wheel's diameter and wheel base
const float DIST_PER_TICK = (M_PI * 4.0 * 0.0254 / (20 * 125));  // pi * diameter * meters_to_inches / counts per rev
const float WHEEL_BASE = 7 * 0.0254; // TODO: Get measurement
const int MOTORA_COUNTER_KEY = 0;
const int MOTORB_COUNTER_KEY = 1;

int motorA_stateA_prev;
int motorB_stateA_prev;
int motorA_stateA;
int motorB_stateA;
int motorA_counter = 0;
int motorB_counter = 0;
int motorA_Acounter = 0;
int motorB_Acounter = 0;
int motorA_counter_prev = 0;
int motorB_counter_prev = 0;
int direction_right = 1;
int direction_left = 1;
float theta = M_PI_2;  // In radians
float x = 0;      // m TODO: get starting position
float y = 0;      // m TODO: get starting position

void setup()
{
    wiringPiSetupGpio();
    pinMode(MOTORA_ENCA_PIN, INPUT);
    pinMode(MOTORA_ENCB_PIN, INPUT);
    pinMode(MOTORB_ENCA_PIN, INPUT);
    pinMode(MOTORB_ENCB_PIN, INPUT);

    motorA_stateA_prev = digitalRead(MOTORA_ENCA_PIN);
    motorB_stateA_prev = digitalRead(MOTORB_ENCA_PIN);
}

PI_THREAD(motorA_encoder)
{
    ROS_INFO("Thread A started!");
    while (1)
    {
        // TODO: Check increment/decrement since the encoders are mounted in different directions for the two motors
        motorA_stateA = digitalRead(MOTORA_ENCA_PIN);

        // Check if a state change has occurred
        if (motorA_stateA != motorA_stateA_prev)
        {
            piLock(MOTORA_COUNTER_KEY);
            motorA_Acounter++;
            // If the states are different then the encoder is rotating clockwise
            /*if (digitalRead(MOTORA_ENCB_PIN) != motorA_stateA)
            {
                motorA_counter++;
            }
            else
            {
                motorA_counter--;
            }*/
            piUnlock(MOTORA_COUNTER_KEY);
            motorA_stateA_prev = motorA_stateA;
        }

        delayMicroseconds(30);  // To soften the load on the processor
    }
}

PI_THREAD(motorB_encoder)
{
    ROS_INFO("Thread B started!");

    while (1)
    {
        // TODO: Check increment/decrement since the encoders are mounted in different directions for the two motors
        motorB_stateA = digitalRead(MOTORB_ENCA_PIN);

        // Check if a state change has occurred
        if (motorB_stateA != motorB_stateA_prev)
        {
            piLock(MOTORB_COUNTER_KEY);
            motorB_Acounter++;
            // If the states are different then the encoder is rotating clockwise
           /* if (digitalRead(MOTORB_ENCB_PIN) != motorB_stateA)
            {
                motorB_counter++;
            }
            else
            {
                motorB_counter--;
            }*/
            piUnlock(MOTORB_COUNTER_KEY);
            motorB_stateA_prev = motorB_stateA;
        }

        delayMicroseconds(30);  // To soften the load on the processor
    }
}

void motorCallback(const bill_msgs::MotorCommands::ConstPtr& msg)
{
    if (msg->command == bill_msgs::MotorCommands::TURN)
    {
        float heading_delta = angles::from_degrees(msg->heading) - theta;

        // Keep heading error centered at 0 between -180 and 180
        if (heading_delta > M_PI)
        {
            heading_delta -= 2*M_PI;
        }
        else if (heading_delta < -1*M_PI)
        {
            heading_delta += 2*M_PI;
        }

        if (heading_delta >= 0)
        {
            direction_right = 1;
            direction_left = -1;
        }
        else
        {
            direction_right = -1;
            direction_left = 1;
        }
    }
    else if(msg->command == bill_msgs::MotorCommands::DRIVE)
    {
        if (msg->speed >= 0)
        {
            direction_right = 1;
            direction_left = 1;
        }
        else
        {
            direction_right = -1;
            direction_left = -1;
        }
    }
}

nav_msgs::Odometry calculateOdometry(float dt)
{
    // TODO: Account and determine slip coeff, radius inequalities and wheelbase errors
    piLock(MOTORA_COUNTER_KEY);
    piLock(MOTORB_COUNTER_KEY);
    int delta_right = direction_right * (motorA_Acounter - motorA_counter_prev);
    int delta_left = direction_left * (motorB_Acounter - motorB_counter_prev);
    ROS_INFO("Delta Right: %i, Delta Left: %i", delta_right, delta_left);
    //motorA_counter = 0;
    //motorB_counter = 0;
    //ROS_INFO("RightA: %i, LeftA: %i", motorA_Acounter, motorB_Acounter);
    motorA_Acounter = 0;
    motorB_Acounter = 0;
    motorA_counter_prev = motorA_Acounter;
    motorB_counter_prev = motorB_Acounter;
    piUnlock(MOTORA_COUNTER_KEY);
    piUnlock(MOTORB_COUNTER_KEY);

    float v_right = delta_right * DIST_PER_TICK / dt;
    float v_left = delta_left * DIST_PER_TICK / dt;

    float v_robot = (v_right + v_left) / 2.0;
    float v_th = (v_right - v_left) / WHEEL_BASE;  // rad/s

    float delta_x = v_robot * cos(theta) * dt;  // These calcs are done with the prev theta value, should be fine though
    float delta_y = v_robot * sin(theta) * dt;
    float delta_th = v_th * dt;

    x += delta_x;
    y += delta_y;
    theta += delta_th;

    if (theta >= 2 * M_PI)
    {
        theta -= 2 * M_PI;
    }
    else if (theta < 0)
    {
        theta += 2 * M_PI;
    }

    nav_msgs::Odometry msg;
    msg.header.frame_id = "odom";
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    msg.pose.covariance = {0.3, 0, 0, 0, 0, 0,
                           0, 0.3, 0, 0, 0, 0,
                           0, 0, -1, 0, 0, 0,
                           0, 0, 0, -1, 0, 0,
                           0, 0, 0, 0, -1, 0,
                           0, 0, 0, 0, 0, 0.1};
    msg.child_frame_id  = "base_link";
    msg.twist.twist.linear.x = v_robot;
    msg.twist.twist.linear.y = 0;
    msg.twist.twist.linear.z = 0;
    msg.twist.twist.angular.z = v_th;
    msg.twist.covariance = {0.02, 0, 0, 0, 0, 0,
                           0, -1, 0, 0, 0, 0,
                           0, 0, -1, 0, 0, 0,
                           0, 0, 0, -1, 0, 0,
                           0, 0, 0, 0, -1, 0,
                           0, 0, 0, 0, 0, 0.01};

    ROS_INFO("Heading: %f", angles::to_degrees(theta));

    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_driver");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odometry", 100);
    ros::Rate loop_rate(LOOP_RATE_ENCODER);
    ros::Subscriber motor_sub = nh.subscribe("motor_cmd", 1, motorCallback);

    // Call sensor setup
    setup();

    // Setup encoder polling threads
    int threadA = piThreadCreate(motorA_encoder);
    if (threadA != 0)
    {
        ROS_ERROR("Thread A did not start!");
    }

    int threadB = piThreadCreate(motorB_encoder);
    if (threadB != 0)
    {
        ROS_ERROR("Thread B did not start!");
    }

    while (ros::ok())
    {
        nav_msgs::Odometry msg = calculateOdometry(loop_rate.expectedCycleTime().toSec());

        // Publish message, and spin thread
        odom_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}