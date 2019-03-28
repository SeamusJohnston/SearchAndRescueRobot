#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "wiringPi.h"
#include "bill_drivers/constant_definition.hpp"
#include <iostream>
#include <limits>  // for NaN
#include <chrono>
#include "bill_drivers/filters.hpp"
#include "bill_msgs/MotorCommands.h"

const static int ECHO_RECEIVE_TIMEOUT = 30000;
const static int ECHO_READ_TIMEOUT = 15000;
const static float DISTANCE_SCALE_CM = 57.0;
// Assumes speed of sound is 340 m/s
// The below value must be converted to cm/microsecond
// const static float SPEED_OF_SOUND_CMPUS = 0.0340;
int trig_pin;
int echo_pin;
bool first_msg = true;
bool turning = false;
auto last_msg_time = std::chrono::high_resolution_clock::now();
LowPassFilter lp_filter;

void setup()
{
    // Init GPIO
    wiringPiSetupGpio();

    // Set up pins
    pinMode(trig_pin, OUTPUT);
    pinMode(echo_pin, INPUT);
    ROS_INFO("Trigger pin is: %i", trig_pin);
    ROS_INFO("Echo pin is: %i", echo_pin);

    // Init trigger as low
    digitalWrite(trig_pin, LOW);
}

float readDistance()
{
    // Make sure the trigger pin starts low
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(4);

    // Send pulse of 10 microseconds
    digitalWrite(trig_pin, HIGH);
    // Note this delay MUST be longer than 10 microseconds
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    // Wait for echo to return
    int receiveTimeout = ECHO_RECEIVE_TIMEOUT;

    while (digitalRead(echo_pin) == LOW)
    {
        if (--receiveTimeout == 0)
        {
            ROS_WARN("Ultrasonic sensor never received echo");
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    // Echo has been detected, measure how long its high for
    // *High by Young Thug and Elton John plays softly in the background*

    //int readTimeout = ECHO_READ_TIMEOUT;
    long startTime = micros();

    while (digitalRead(echo_pin) == HIGH)
    {
        if (micros() - startTime > ECHO_READ_TIMEOUT)
        {
            ROS_WARN("Ultrasonic sensor timed out while reading echo");
            ROS_INFO("TIME: %i", micros() - startTime);
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    long echoTime = micros() - startTime;

    return (echoTime / DISTANCE_SCALE_CM);
}

std_msgs::Float32 read()
{
    float distance = readDistance();

    if (!std::isnan(distance) && !turning)
    {
        auto time_now = std::chrono::high_resolution_clock::now();
        if (!first_msg)
        {
            lp_filter.setSamplingTime(std::chrono::duration<float>(time_now - last_msg_time).count());
            distance = lp_filter.update(distance);
        }
        else
        {
            lp_filter.setPrevOutput(distance);
            first_msg = false;
        }

        last_msg_time = time_now;
    }
    else
    {
        ROS_WARN("Error on ultrasonic");
    }

    // Create message to publish
    std_msgs::Float32 msg;
    // Populate the message
    msg.data = distance;

    return msg;
}

void motorCallback(const bill_msgs::MotorCommands::ConstPtr& msg)
{
    if (msg->command == bill_msgs::MotorCommands::TURN)
    {
        turning = true;
    }
    else
    {
        if (turning) // Was just turning but has now stopped, then reset the filters
        {
            first_msg = true;

        }
        turning = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasonic_driver");
    ros::NodeHandle nh("~");
    std::string topic;
    nh.getParam("trigger_pin", trig_pin);
    nh.getParam("echo_pin", echo_pin);
    nh.getParam("topic", topic);
    float freq = 0;
    nh.getParam("filter_freq", freq);
    lp_filter.setFrequency(freq);

    ros::Subscriber motor_sub = nh.subscribe("/motor_cmd", 1, motorCallback);
    ros::Publisher ultrasonic_pub = nh.advertise<std_msgs::Float32>(topic, 100);
    ros::Rate loop_rate(LOOP_RATE_ULTRA);

    // Call sensor setup
    setup();

    while (ros::ok())
    {
        std_msgs::Float32 msg = read();

        // Publish message if valid, and spin thread
        if (!std::isnan(msg.data) && !turning)
            ultrasonic_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
