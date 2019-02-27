#include "ros/ros.h"
#include "wiringPi.h"
#include "std_msgs/Bool.h"

#define BUTTON_PIN 20

void setup()
{
    // Init GPIO
    wiringPiSetupGpio();

    pinMode(BUTTON_PIN, INPUT);
    pullUpDnControl(BUTTON_PIN, PUD_UP);
    ROS_INFO("Reset pin is: %i, and set up to PULL UP", BUTTON_PIN);
}

std_msgs::Bool read()
{
    // Create message to publish
    std_msgs::Bool msg;

    // Populate the message
    msg.data = (digitalRead(BUTTON_PIN) == LOW);

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

            ROS_INFO("Reset Button Status: %s", msg.data ? "Pressed" : "Not Pressed");

            ros::spinOnce();
        }
        past_reset.data = msg.data;
        loop_rate.sleep();
    }
    return 0;
}
