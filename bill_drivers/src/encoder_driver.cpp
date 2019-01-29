#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

void setup()
{
    // TODO: Setup sensor
}

nav_msgs::Odometry read()
{
    // TODO: Read sensor

    // Create message to publish
    nav_msgs::Odometry msg;
    // TODO: Based on read data, determine what to publish

    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_driver");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Rate loop_rate(10);

    // Call sensor setup
    setup();

    while (ros::ok())
    {
        nav_msgs::Odometry msg = read();

        // Publish message, and spin thread
        odom_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
