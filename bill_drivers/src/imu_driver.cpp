#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "constants.hpp"

void setup()
{
    // TODO: Setup sensor
}

sensor_msgs::Imu read()
{
    // TODO: Read sensor

    // Create message to publish
    sensor_msgs::Imu msg;
    // TODO: Populate the message

    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_driver");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
    ros::Rate loop_rate(LOOP_RATE_IMU);

    // Call sensor setup
    setup();

    while (ros::ok())
    {
        sensor_msgs::Imu msg = read();

        // Publish message, and spin thread
        imu_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
