#include "bill_msgs/Survivor.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
#include "bill_drivers/pi2c.h"
#include "constants.hpp"
#include <bitset>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "i2c_driver");
    ros::NodeHandle nh;
    ros::Publisher survivor_pub = nh.advertise<bill_msgs::Survivor>("survivors", 100);
    ros::Publisher food_pub = nh.advertise<std_msgs::Bool>("food", 100);
    ros::Publisher fire_pub = nh.advertise<std_msgs::Bool>("fire", 100);
    ros::Rate loop_rate(LOOP_RATE_I2C);

    // Create i2c object for arduino
    Pi2c arduino(7);
    char data[1];
    std::bitset<8> fire_bitmap = 0x01;
    std::bitset<8> food_bitmap = 0x02;
    std::bitset<8> survivor_bitmap = 0x0C;

    while (ros::ok())
    {
        bill_msgs::Survivor survivor_msg;
        std_msgs::Bool food_msg;
        std_msgs::Bool fire_msg;
        // Receive from the Arduino and put the contents into the "receive" char array
        arduino.i2cRead(data, 1);
        std::bitset<8> data_received(data[0]);
        // Print out what the Arduino is sending...
        ROS_INFO("Received Data from Arduino: %i\n", (int)data_received.to_ulong());

        // Parse the byte, bits 3 and 2 are survivor data, 1 is food and 0 is fire
        survivor_msg.data = (int)((data_received & survivor_bitmap) >> 2).to_ulong();
        food_msg.data = (bool)(data_received & food_bitmap).to_ulong();
        fire_msg.data = (bool)(data_received & fire_bitmap).to_ulong();

        // Publish message, and spin thread
        survivor_pub.publish(survivor_msg);
        food_pub.publish(food_msg);
        fire_pub.publish(fire_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
