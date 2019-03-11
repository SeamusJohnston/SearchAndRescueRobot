#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "bill_msgs/MotorCommands.h"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_drivers/constant_definition.hpp"
#include "bill_drivers/filters.hpp"

const float TOL = 10.0;
const float COURSE_DIM = 1.8;
const float ROBOT_WIDTH = 0.20;

struct Position
{
    float x;
    float y;
};

float front_dist = 0;
float right_dist = 0;
float left_dist = 0;
Position odom_pos;
Position ultra_pos;
int current_heading;
bool turning = false;
ComplementaryFilter comp_filter_x(1.0, 0.1); // TODO: figure out sampling time
ComplementaryFilter comp_filter_y(1.0, 0.1); // TODO: figure out sampling time
LowPassFilter lp_right(1.0, 0.1);
LowPassFilter lp_left(1.0, 0.1);
LowPassFilter lp_front(1.0, 0.1);

void updatePosition()
{
    float x = comp_filter_x.update(odom_pos.x, ultra_pos.x);
    float y = comp_filter_y.update(odom_pos.y, ultra_pos.y);

    ROS_INFO("Current filtered position x: %f, y: %f", x, y);
    ROS_INFO("Current ultrasonic position x: %f, y: %f", ultra_pos.x, ultra_pos.y);
    // TODO: Publish here
}

void updateSideDist()
{
    float expected_right = 0;
    float expected_left = 0;

    // TODO: Depending on the angle, use side dist to calculate either x or y position absolutely
    // Only if it adds to the expected value though
    // We could say that if there is an obstacle then we add the delta position in whichever direction from the encoders
    // But since we are always travelling parallel to the walls it should not make a huge difference.

    updatePosition();
}

void updateFrontDist()
{
    // TODO: Depending on the angle, use front dist to calculate either x or y position absolutely or relatively
    // If current_ultra_pos + front_dist == expected_vale +- TOL then use absolute positioning
        // y = 1.8 - front_pos - robot_length/2.0;
    // else then use relative positioning (could find a datum or something) but currently do this
        // y = ultra_pos + (last_front_dist - front_dist)

    updatePosition();
}

void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    front_dist = lp_front.update(msg->data/100.0);
    if (!turning)
        updateFrontDist();
}

void leftUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    left_dist = lp_left.update(msg->data/100.0);
    if (!turning)
        updateSideDist();
}

void rightUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    right_dist = lp_right.update(msg->data/100.0);
    if (!turning)
        updateSideDist();
}

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_pos.x = msg->pose.pose.position.x;
    odom_pos.y = msg->pose.pose.position.y;
    current_heading = (int)angles::to_degrees(tf::getYaw(msg->pose.pose.orientation));
}

void motorCallback(const bill_msgs::MotorCommands::ConstPtr& msg)
{
    if (msg->command == bill_msgs::MotorCommands::TURN)
        turning = true;
    else
        turning = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_front = nh.subscribe("ultra_front", 1, frontUltrasonicCallback);
    ros::Subscriber sub_left = nh.subscribe("ultra_left", 1, leftUltrasonicCallback);
    ros::Subscriber sub_right = nh.subscribe("ultra_right", 1, rightUltrasonicCallback);
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 1, fusedOdometryCallback);
    ros::Subscriber sub_motors = nh.subscribe("motor_cmd", 1, motorCallback);
    ros::spin();
    return 0;
}
