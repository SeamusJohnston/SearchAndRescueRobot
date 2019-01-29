#include "ros/ros.h"
#include "std_msgs/Bool.h"

void ledCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        // TODO: Turn on led
    }
    else
    {
        // TODO: Turn off led
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "led_driver");
    ros::NodeHandle nh;
    ros::Subscriber sub_led = nh.subscribe("led", 1, ledCallback);

    ros::spin();
    return 0;
}
