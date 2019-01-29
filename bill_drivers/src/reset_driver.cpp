#include "ros/ros.h"
#include "std_msgs/Bool.h"

void setup()
{
    // TODO: Setup sensor
}

std_msgs::Bool read()
{
    // TODO: Read sensor

    // Create message to publish
    std_msgs::Bool msg;
    // TODO: Populate the message

    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reset_driver");
    ros::NodeHandle nh;
    ros::Publisher reset_pub = nh.advertise<std_msgs::Bool>("reset", 100);
    ros::Rate loop_rate(10);

    // Call sensor setup
    setup();
    std_msgs::Bool past_reset;
    past_reset.data = false;

    while (ros::ok())
    {
        std_msgs::Bool msg = read();

        // To avoid unneccesary publications, only publish if there is a state
        // change
        if (msg.data != past_reset.data)
        {
            // Publish message, and spin thread
            reset_pub.publish(msg);
            ros::spinOnce();
        }
        past_reset.data = msg.data;
        loop_rate.sleep();
    }
    return 0;
}
