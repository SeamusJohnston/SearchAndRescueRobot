#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "wiringPi.h"
#include "angles/angles.h"
#include "bill_drivers/constant_definition.hpp"
#include <cmath>

// TODO: Get an accurate measurement of the wheel's diameter and wheel base
const float DIST_PER_TICK = (M_PI * 2.0 * 0.0254 / (20 * 125));  // pi * diameter * meters_to_inches / counts per rev
const float WHEEL_BASE = 5.76; // TODO: Get measurement
const int MOTORA_COUNTER_KEY = 0;
const int MOTORB_COUNTER_KEY = 1;

int motorA_stateA_prev;
int motorB_stateA_prev;
int motorA_stateA;
int motorB_stateA;
int motorA_counter = 0;
int motorB_counter = 0;
int motorA_counter_prev = 0;
int motorB_counter_prev = 0;
float theta = 0;  // In radians TODO: Determine what our starting orientation is
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
            // If the states are different then the encoder is rotating clockwise
            if (digitalRead(MOTORA_ENCB_PIN) != motorA_stateA)
            {
                motorA_counter++;
            }
            else
            {
                motorA_counter--;
            }
            piUnlock(MOTORA_COUNTER_KEY);
            motorA_stateA_prev = motorA_stateA;
        }

        delayMicroseconds(2);  // To soften the load on the processor
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
            // If the states are different then the encoder is rotating clockwise
            if (digitalRead(MOTORB_ENCB_PIN) != motorB_stateA)
            {
                motorB_counter++;
            }
            else
            {
                motorB_counter--;
            }
            piUnlock(MOTORB_COUNTER_KEY);
            motorB_stateA_prev = motorB_stateA;
        }

        delayMicroseconds(2);  // To soften the load on the processor
    }
}

nav_msgs::Odometry calculateOdometry(float dt)
{
    // TODO: Account and determine slip coeff, radius inequalities and wheelbase errors
    piLock(MOTORA_COUNTER_KEY);
    piLock(MOTORB_COUNTER_KEY);
    int delta_right = motorA_counter - motorA_counter_prev;
    int delta_left = motorB_counter - motorB_counter_prev;
    motorA_counter = 0;
    motorB_counter = 0;
    motorA_counter_prev = motorA_counter;
    motorB_counter_prev = motorB_counter;
    piUnlock(MOTORA_COUNTER_KEY);
    piUnlock(MOTORB_COUNTER_KEY);

    float v_right = delta_right * DIST_PER_TICK;
    float v_left = delta_left * DIST_PER_TICK;

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
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    msg.twist.twist.linear.x = delta_x / dt;
    msg.twist.twist.linear.y = delta_y / dt;
    msg.twist.twist.linear.z = 0;
    msg.twist.twist.angular.z = v_th;

    ROS_INFO("Heading: %f", angles::to_degrees(theta));

    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_driver");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Rate loop_rate(LOOP_RATE_ENCODER);

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
