#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "bill_msgs/MotorCommands.h"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_drivers/constant_definition.hpp"
#include "bill_drivers/filters.hpp"

const float TOL = 5.0; // TODO: Determine appropriate range
const float COURSE_DIM = 1.8; // TODO: Measure
const float ROBOT_WIDTH = 0.20; // TODO: Measure
const float ROBOT_LENGTH = 0.30; // TODO: Measure

struct Position
{
    Position(float xin, float yin): x(xin), y(yin){};
    float x;
    float y;
};

float front_dist = 0;
float prev_front_dist = -1;
float right_dist = 0;
float left_dist = 0;
Position odom_pos(0,0); // TODO: Determine start position
Position ultra_pos(0,0); // TODO: Determine start position
int current_heading = 90;
bool turning = false;
ComplementaryFilter comp_filter_x(1.0, 0.1); // TODO: figure out sampling time, frequency
ComplementaryFilter comp_filter_y(1.0, 0.1); // TODO: figure out sampling time, frequency
LowPassFilter lp_right(1.0, 0.1); // TODO: figure out sampling time, frequency
LowPassFilter lp_left(1.0, 0.1); // TODO: figure out sampling time, frequency
LowPassFilter lp_front(1.0, 0.1); // TODO: figure out sampling time, frequency

float cosDegrees(int angle)
{
    return std::cos(angles::from_degrees(angle));
}

void updatePosition()
{
    float x = comp_filter_x.update(odom_pos.x, ultra_pos.x);
    float y = comp_filter_y.update(odom_pos.y, ultra_pos.y);

    ROS_INFO("Current filtered position x: %f, y: %f", x, y);
    ROS_INFO("Current ultrasonic position x: %f, y: %f", ultra_pos.x, ultra_pos.y);
    // TODO: Publish here
    // TODO: re-update ultra pos with filtered values?
}

void updateSideDist()
{
    if (45 < current_heading && current_heading <= 135) // Facing positive y
    {
        float expected_dist = COURSE_DIM / cosDegrees(90 - current_heading) - ROBOT_WIDTH;
        if (std::abs(expected_dist - (right_dist + left_dist)) < 2*TOL) // Distance is valid, update pos
        {
            ultra_pos.x = (left_dist * cosDegrees(90 - current_heading) + (COURSE_DIM - right_dist * cosDegrees(90 - current_heading))) / 2.0; // Average * cos(theta)
        }
    }
    else if (135 < current_heading && current_heading <= 225) // Facing negative x
    {
        float expected_dist = COURSE_DIM / cosDegrees(current_heading - 180) - ROBOT_WIDTH;
        if (std::abs(expected_dist - (right_dist + left_dist)) < 2*TOL) // Distance is valid, update pos
        {
            ultra_pos.y = (left_dist * cosDegrees(current_heading - 180) + (COURSE_DIM - right_dist * cosDegrees(current_heading - 180))) / 2.0; // Average * cos(theta)
        }
    }
    else if (225 < current_heading && current_heading <= 315) // Facing negative y
    {
        float expected_dist = COURSE_DIM / cosDegrees(270 - current_heading) - ROBOT_WIDTH;
        if (std::abs(expected_dist - (right_dist + left_dist)) < 2*TOL) // Distance is valid, update pos
        {
            ultra_pos.x = ((COURSE_DIM - left_dist * cosDegrees(270 - current_heading)) + right_dist * cosDegrees(270 - current_heading)) / 2.0; // Average * cos*(theta)
        }
    }
    else // Facing positive x
    {
        float expected_dist = COURSE_DIM / cosDegrees(current_heading) - ROBOT_WIDTH;
        if (std::abs(expected_dist - (right_dist + left_dist)) < 2*TOL) // Distance is valid, update pos
        {
            ultra_pos.y = ((COURSE_DIM - left_dist * cosDegrees(current_heading)) + right_dist * cosDegrees(current_heading)) / 2.0; // Average * cos*(theta)
        }
    }

    // We could say that if there is an obstacle then we add the delta position in whichever direction from the encoders
    // But since we are always travelling parallel to the walls it should not make a huge difference.

    updatePosition();
}

void updateFrontDist()
{
    // If current_ultra_pos + front_dist == expected_vale +- TOL then use absolute positioning
    // y = 1.8 - front_pos - robot_length/2.0;
    // else then use relative positioning (could find a datum or something) but currently do this
    // y = ultra_pos + (last_front_dist - front_dist)

    if (45 < current_heading && current_heading <= 135) // Facing positive y
    {
        float expected_dist = (COURSE_DIM - ultra_pos.y) / cosDegrees(90 - current_heading) - ROBOT_LENGTH / 2.0;
        if (std::abs(expected_dist - front_dist) < TOL) // If expected, use absolute positioning
        {
            ultra_pos.y = COURSE_DIM - front_dist * cosDegrees(90 - current_heading) - ROBOT_LENGTH / 2.0;
        }
        else if (prev_front_dist > 0)// Use relative positioning, if we have previous measurement
        {
            ultra_pos.y += prev_front_dist - front_dist;
        }
    }
    else if (135 < current_heading && current_heading <= 225) // Facing negative x
    {
        float expected_dist = ultra_pos.x / cosDegrees(current_heading - 180) - ROBOT_LENGTH / 2.0;
        if (std::abs(expected_dist - front_dist) < TOL) // If expected, use absolute positioning
        {
            ultra_pos.x = front_dist * cosDegrees(current_heading - 180) + ROBOT_LENGTH / 2.0;
        }
        else if (prev_front_dist > 0)// Use relative positioning, if we have previous measurement
        {
            ultra_pos.x -= prev_front_dist - front_dist;
        }
    }
    else if (225 < current_heading && current_heading <= 315) // Facing negative y
    {
        float expected_dist = ultra_pos.y / cosDegrees(270 - current_heading) - ROBOT_LENGTH / 2.0;
        if (std::abs(expected_dist - front_dist) < TOL) // If expected, use absolute positioning
        {
            ultra_pos.y = front_dist * cosDegrees(270 - current_heading) + ROBOT_LENGTH / 2.0;
        }
        else if (prev_front_dist > 0)// Use relative positioning, if we have previous measurement
        {
            ultra_pos.y -= prev_front_dist - front_dist;
        }
    }
    else // Facing positive x
    {
        float expected_dist = (COURSE_DIM - ultra_pos.x) / cosDegrees(current_heading) - ROBOT_LENGTH / 2.0;
        if (std::abs(expected_dist - front_dist) < TOL) // If expected, use absolute positioning
        {
            ultra_pos.x = COURSE_DIM - front_dist * cosDegrees(90 - current_heading) - ROBOT_LENGTH / 2.0;
        }
        else if (prev_front_dist > 0)// Use relative positioning, if we have previous measurement
        {
            ultra_pos.x += prev_front_dist - front_dist;
        }
    }

    prev_front_dist = front_dist;
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
    {
        turning = true;
        prev_front_dist = -1;
    }
    else
    {
        turning = false;
    }

    // TODO: Determine if there is a special case for stop action
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
    // TODO: Add publisher
    ros::spin();
    return 0;
}
