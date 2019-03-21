#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "wiringPi.h"
#include "angles/angles.h"
#include "bill_drivers/constant_definition.hpp"
#include "sensor_msgs/Imu.h"
#include <cmath>

ros::Publisher odom_pub;
ros::Publisher imu_pub;
bool first_msg = true;
tf::Quaternion initial_transformation;

void odometryCallback(const nav_msgs::Odometry msg)
{
 /*   nav_msgs::Odometry new_msg = msg;
    new_msg.header.stamp = ros::Time::now();
    odom_pub.publish(new_msg);*/
    //std::cout << angles::to_degrees(tf::getYaw(msg.pose.pose.orientation)) << std::endl;
}

void fusedCallback(const nav_msgs::Odometry msg)
{
    //nav_msgs::Odometry new_msg = msg;
    //new_msg.header.stamp = ros::Time::now();
    //odom_pub.publish(new_msg);
    //std::cout << angles::to_degrees(tf::getYaw(msg.pose.pose.orientation)) << std::endl;
}

void imuCallback(const sensor_msgs::Imu msg)
{
 /*   sensor_msgs::Imu new_msg = msg;
    if (first_msg)
    {
        initial_transformation.setRPY(0,0,-tf::getYaw(msg.orientation)+M_PI_2);
        first_msg = false;
    }
    new_msg.header.stamp = ros::Time::now();
    tf::Quaternion quat;
    tf::quaternionMsgToTF(new_msg.orientation, quat);
    quat = (quat * initial_transformation).normalize();
    new_msg.orientation.x = quat.x();
    new_msg.orientation.y = quat.y();
    new_msg.orientation.z = quat.z();
    new_msg.orientation.w = quat.w();

    imu_pub.publish(new_msg);*/
    std::cout << angles::to_degrees(tf::getYaw(msg.orientation)) << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("test_odometry", 100);
    imu_pub = nh.advertise<sensor_msgs::Imu>("test_imu", 100);
    ros::Subscriber sub_imu = nh.subscribe("test_imu", 10, imuCallback);
    ros::Subscriber sub_odom = nh.subscribe("odometry", 10, odometryCallback);
    ros::Subscriber sub_fusedOdom = nh.subscribe("filtered_odometry", 10, fusedCallback);
    ros::spin();

    return 0;
}
