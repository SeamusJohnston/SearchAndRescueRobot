#include "bill_msgs/Survivor.h"
#include "ros/ros.h"

void setup()
{
    // TODO: Setup sensor
}

bill_msgs::Survivor read()
{
    // TODO: Read sensor

    // Create message to publish
    bill_msgs::Survivor msg;
    // TODO: Based on read data, determine what to publish

    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "colour_driver");
    ros::NodeHandle nh;
    ros::Publisher survivor_pub = nh.advertise<bill_msgs::Survivor>("survivors", 100);
    ros::Rate loop_rate(10);

    // Call sensor setup
    setup();

    while (ros::ok())
    {
        bill_msgs::Survivor msg = read();

        // Publish message, and spin thread
        survivor_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
