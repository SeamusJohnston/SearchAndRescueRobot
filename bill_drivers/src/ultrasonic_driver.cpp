#include "ros/ros.h"
#include "std_msgs/Float32.h"

void setup()
{
    // TODO: Setup sensor
}

std_msgs::Float32 read()
{
    // TODO: Read sensor

    // Create message to publish
    std_msgs::Float32 msg;
    // TODO: Populate the message

    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasonic_driver");
    ros::NodeHandle nh;
    ros::Publisher ultrasonic_pub = nh.advertise<std_msgs::Float32>("ultrasonic", 100);
    ros::Rate loop_rate(10);

    // Call sensor setup
    setup();

    while (ros::ok())
    {
        std_msgs::Float32 msg = read();

        // Publish message, and spin thread
        ultrasonic_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
