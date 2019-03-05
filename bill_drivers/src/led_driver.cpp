#include "ros/ros.h"
#include "wiringPi.h"
#include "std_msgs/Bool.h"
#include "bill_drivers/constant_definition.hpp"

void setup()
{
    // Init GPIO
    wiringPiSetupGpio();

    // Set up pins
    pinMode(LED_PIN, OUTPUT);
    ROS_INFO("LED pin is: %i", LED_PIN);
}

void ledCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        // Turn on led
        digitalWrite(LED_PIN, HIGH);
    }
    else
    {
        // Turn off led
        digitalWrite(LED_PIN, LOW);
    }
}

int main(int argc, char** argv)
{
    // Set up LED
    setup();

    ros::init(argc, argv, "led_driver");
    ros::NodeHandle nh;
    ros::Subscriber sub_led = nh.subscribe("led", 1, ledCallback);

    ros::spin();
    return 0;
}
