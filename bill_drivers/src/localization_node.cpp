#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "bill_msgs/MotorCommands.h"
#include "bill_msgs/Position.h"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_drivers/constant_definition.hpp"
#include "bill_drivers/filters.hpp"
#include <limits>
#include <chrono>

const float TOL = 0.05; // TODO: Determine appropriate range
const float COURSE_DIM = 1.85;
const float ROBOT_WIDTH = 7.0 * 0.0254;
const float ROBOT_LENGTH = 0.26;
const int buffer_size = 5;

struct Position
{
    Position(float xin, float yin): x(xin), y(yin){};
    float x;
    float y;
};

//float front_dist = 0;
//float prev_front_dist = -1;
float right_dist = 0;
float left_dist = 0;

//auto last_comp_msg = std::chrono::high_resolution_clock::now();
//bool first_comp_msg = true;

Position odom_pos(1.05, 0.15);
Position odom_pos_prev(1.05, 0.15);
Position ultra_pos(1.05, 0.15);
float side_buffer[buffer_size] = {0};
int buffer_counter = 0;
int current_heading = 90;
bool turning = false;
//ComplementaryFilter comp_filter_x(2.0); // TODO: figure out frequency
//ComplementaryFilter comp_filter_y(2.0); // TODO: figure out frequency
ros::Publisher position_pub;
//ros::Publisher odom_ultra_pub;

float cosDegrees(int angle)
{
    return std::cos(angles::from_degrees(angle));
}

void publishPosition()
{
    bill_msgs::Position msg;
    msg.heading = current_heading;
    msg.x = ultra_pos.x;
    msg.y = ultra_pos.y;
    position_pub.publish(msg);
/*
    nav_msgs::Odometry msg_odom;
    msg_odom.header.stamp = ros::Time::now();
    msg_odom.header.frame_id = "odom";
    msg_odom.pose.pose.position.x = ultra_pos.x;
    msg_odom.pose.pose.position.y = ultra_pos.y;
    if ((45 < current_heading && current_heading <= 135) || (225 < current_heading && current_heading <= 315)) // Facing y
    {
        msg_odom.pose.covariance = {0.02, 0, 0, 0, 0, 0,
                                    0, 0.05, 0, 0, 0, 0,
                                    0, 0, -1, 0, 0, 0,
                                    0, 0, 0, -1, 0, 0,
                                    0, 0, 0, 0, -1, 0,
                                    0, 0, 0, 0, 0, -1};
    }
    else
    {
        msg_odom.pose.covariance = {0.05, 0, 0, 0, 0, 0,
                                    0, 0.02, 0, 0, 0, 0,
                                    0, 0, -1, 0, 0, 0,
                                    0, 0, 0, -1, 0, 0,
                                    0, 0, 0, 0, -1, 0,
                                    0, 0, 0, 0, 0, -1};
    }
    odom_ultra_pub.publish(msg_odom);
    */
}

void updatePosition()
{
/*
    auto time_now = std::chrono::high_resolution_clock::now();
    if (!first_comp_msg)
    {
        comp_filter_x.setSamplingTime(std::chrono::duration<float>(time_now - last_comp_msg).count());
        comp_filter_y.setSamplingTime(std::chrono::duration<float>(time_now - last_comp_msg).count());
        //ultra_pos.x = comp_filter_x.update(odom_pos.x, ultra_pos.x);
        //ultra_pos.y = comp_filter_y.update(odom_pos.y, ultra_pos.y);
        // TODO: re-update ultra pos with filtered values?
        //ROS_INFO("Current filtered position x: %f, y: %f", x, y);
    }
    else
    {
        comp_filter_x.setPrevOutput(odom_pos.x, ultra_pos.x); // Reset filter to maintain continuity
        comp_filter_y.setPrevOutput(odom_pos.y, ultra_pos.y);
        first_comp_msg = false;
    }

    last_comp_msg = time_now;
*/
    // Check if an update in side position can be performed
    // Average the buffer
    if(!turning)
    {
        float sum = 0;
        for (int i = 0; i < buffer_size; i++)
        {
            sum += side_buffer[i];
        }
        float average = sum / buffer_size;

        // Check the max variance, if it exceeds a certain threshold, get don't update the estimate
        bool side_update = true;
        for (int i = 0; i < buffer_size; i++) {
            if (std::abs(average - side_buffer[i]) > TOL || side_buffer[i] == 0) {
                side_update = false;
                break;
            }
        }

        if ((45 < current_heading && current_heading <= 135) ||
            (225 < current_heading && current_heading <= 315)) // Facing y
        {
            ultra_pos.y += odom_pos.y - odom_pos_prev.y;
            odom_pos_prev.y = odom_pos.y; // Set to current so next time this loop is called the delta will be zero if no new data
            if (side_update && ultra_pos.x != average) {
                ROS_INFO("Ultrasonic side update X position from %f, to %f", ultra_pos.x, average);
                ultra_pos.x = average;
            }
        } else {
            ultra_pos.x += odom_pos.x - odom_pos_prev.x;
            odom_pos_prev.x = odom_pos.x; // Set to current so next time this loop is called the delta will be zero if no new data
            if (side_update && ultra_pos.y != average) {
                ROS_INFO("Ultrasonic side update Y position from %f, to %f", ultra_pos.y, average);
                ultra_pos.y = average;
            }
        }
    }

    ROS_INFO("Current ultrasonic position x: %f, y: %f, Heading: %i, Turning: %i", ultra_pos.x, ultra_pos.y, current_heading, turning);
    publishPosition();
}

void updateSideDist()
{
    if (45 < current_heading && current_heading <= 135) // Facing positive y
    {
        float expected_dist = COURSE_DIM / cosDegrees(90 - current_heading) - ROBOT_WIDTH;
        if (std::abs(expected_dist - (right_dist + left_dist)) < 2*TOL) // Distance is valid, update pos
        {
            side_buffer[buffer_counter] = (left_dist * cosDegrees(90 - current_heading) + (COURSE_DIM - right_dist * cosDegrees(90 - current_heading))) / 2.0; // Average * cos(theta)
            buffer_counter = (buffer_counter + 1) % buffer_size;
        }
    }
    else if (135 < current_heading && current_heading <= 225) // Facing negative x
    {
        float expected_dist = COURSE_DIM / cosDegrees(current_heading - 180) - ROBOT_WIDTH;
        if (std::abs(expected_dist - (right_dist + left_dist)) < 2*TOL) // Distance is valid, update pos
        {
            side_buffer[buffer_counter] = (left_dist * cosDegrees(current_heading - 180) + (COURSE_DIM - right_dist * cosDegrees(current_heading - 180))) / 2.0; // Average * cos(theta)
            buffer_counter = (buffer_counter + 1) % buffer_size;
        }
    }
    else if (225 < current_heading && current_heading <= 315) // Facing negative y
    {
        float expected_dist = COURSE_DIM / cosDegrees(270 - current_heading) - ROBOT_WIDTH;
        if (std::abs(expected_dist - (right_dist + left_dist)) < 2*TOL) // Distance is valid, update pos
        {
            side_buffer[buffer_counter] = ((COURSE_DIM - left_dist * cosDegrees(270 - current_heading)) + right_dist * cosDegrees(270 - current_heading)) / 2.0; // Average * cos*(theta)
            buffer_counter = (buffer_counter + 1) % buffer_size;
        }
    }
    else // Facing positive x
    {
        float expected_dist = COURSE_DIM / cosDegrees(current_heading) - ROBOT_WIDTH;
        if (std::abs(expected_dist - (right_dist + left_dist)) < 2*TOL) // Distance is valid, update pos
        {
            side_buffer[buffer_counter] = ((COURSE_DIM - left_dist * cosDegrees(current_heading)) + right_dist * cosDegrees(current_heading)) / 2.0; // Average * cos*(theta)
            buffer_counter = (buffer_counter + 1) % buffer_size;
        }
    }

    // We could say that if there is an obstacle then we add the delta position in whichever direction from the encoders
    // But since we are always travelling parallel to the walls it should not make a huge difference.

    updatePosition();
}
/*
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
            ultra_pos.y += (prev_front_dist - front_dist) * cosDegrees(90 - current_heading);
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
            ultra_pos.x -= (prev_front_dist - front_dist) * cosDegrees(current_heading - 180);
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
            ultra_pos.y -= (prev_front_dist - front_dist) * cosDegrees(270 - current_heading);
        }
    }
    else // Facing positive x
    {
        float expected_dist = (COURSE_DIM - ultra_pos.x) / cosDegrees(current_heading) - ROBOT_LENGTH / 2.0;
        if (std::abs(expected_dist - front_dist) < TOL) // If expected, use absolute positioning
        {
            ultra_pos.x = COURSE_DIM - front_dist * cosDegrees(current_heading) - ROBOT_LENGTH / 2.0;
        }
        else if (prev_front_dist > 0)// Use relative positioning, if we have previous measurement
        {
            ultra_pos.x += (prev_front_dist - front_dist) * cosDegrees(current_heading);
        }
    }

    prev_front_dist = front_dist;
    updatePosition();
}

void frontUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    front_dist = msg->data / 100.0;
    if (!turning)
        updateFrontDist();
}
*/
void leftUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if (!turning)
    {
        left_dist = msg->data / 100.0;
        updateSideDist();
    }
}

void rightUltrasonicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if (!turning)
    {
        right_dist = msg->data / 100.0;
        updateSideDist();
    }
}

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_heading = (int)angles::to_degrees(tf::getYaw(msg->pose.pose.orientation));
    if(current_heading < 0)
    {
        current_heading += 360;
    }
    updatePosition();
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!turning)
    {
        odom_pos_prev.x = odom_pos.x;
        odom_pos_prev.y = odom_pos.y;
        odom_pos.x = msg->pose.pose.position.x;
        odom_pos.y = msg->pose.pose.position.y;
        updatePosition();
    }
}

void motorCallback(const bill_msgs::MotorCommands::ConstPtr& msg)
{
    ROS_INFO("Received motor cmd");
    if (msg->command == bill_msgs::MotorCommands::TURN)
    {
        turning = true;
    }
    else
    {
        if (turning) // Was just turning but has now stopped, then reset the filters and clear previous distances
        {
            //first_comp_msg = true;
            for (int i = 0; i < buffer_size; i++)
            {
                side_buffer[i] = 0;
            }
            buffer_counter = 0;
            //prev_front_dist = -1;

        }
        turning = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;
    //ros::Subscriber sub_front = nh.subscribe("ultra_front", 10, frontUltrasonicCallback);
    ros::Subscriber sub_left = nh.subscribe("ultra_left", 10, leftUltrasonicCallback);
    ros::Subscriber sub_right = nh.subscribe("ultra_right", 10, rightUltrasonicCallback);
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 10, fusedOdometryCallback);
    ros::Subscriber sub_motors = nh.subscribe("motor_cmd", 10, motorCallback);
    ros::Subscriber sub_encoders = nh.subscribe("odometry", 1, odometryCallback);
    position_pub = nh.advertise<bill_msgs::Position>("position", 100);
    //odom_ultra_pub = nh.advertise<nav_msgs::Odometry>("odometry_ultra", 100);

    nh.getParam("/bill/starting_params/x", ultra_pos.x);
    nh.getParam("/bill/starting_params/y", ultra_pos.y);
    nh.getParam("/bill/starting_params/theta", current_heading);
    odom_pos.x = ultra_pos.x;
    odom_pos.y = ultra_pos.y;
    odom_pos_prev.x = ultra_pos.x;
    odom_pos_prev.y = ultra_pos.y;

    ros::spin();
    return 0;
}
