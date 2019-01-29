#include "ros/ros.h"
#include "std_msgs/Bool.h"

void fanCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        // TODO: Turn on fan
    }
    else
    {
        // TODO: Turn off fan
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fan_driver");
    ros::NodeHandle nh;
    ros::Subscriber sub_fan = nh.subscribe("fan", 1, fanCallback);

    ros::spin();
    return 0;
}
