#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "wiringPi.h"
#include "bill_drivers/constant_definition.hpp"
#include <iostream>

void fanCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        digitalWrite(RELAY_OUTPIN, HIGH);
    }
    else
    {
        digitalWrite(RELAY_OUTPIN, LOW);
    }
}

int main(int argc, char** argv)
{
    wiringPiSetupGpio();
    pinMode(RELAY_OUTPIN, OUTPUT);
    ros::init(argc, argv, "fan_driver");
    ros::NodeHandle nh;
    ros::Subscriber sub_fan = nh.subscribe("fan", 1, fanCallback);

    ros::spin();
    return 0;
}
