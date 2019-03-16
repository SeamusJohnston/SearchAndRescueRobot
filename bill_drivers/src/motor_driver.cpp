#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_msgs/MotorCommands.h"
#include "bill_msgs/Position.h"
#include "wiringPi.h"
#include <softPwm.h>
#include "bill_drivers/constant_definition.hpp"
#include <chrono>

const int PWM_RANGE = 100;  // Max pwm value
const int MAX_TURNING_SPEED = 100;
const float INT_CLAMP = 2.0;
const float MAX_VEL = 0.4;
float KP_TURNING;
float KI_TURNING;
float KP_DRIVE;
float KI_DRIVE;

auto last_msg_time = std::chrono::high_resolution_clock::now();
bill_msgs::MotorCommands last_command_msg;
int last_heading = 90;
float heading_error_drive_sum = 0;
float heading_error_turn_sum = 0;


enum Direction
{
    CW = 1,
    CCW = -1
};

void stop()
{
    ROS_INFO("Stop");
    softPwmWrite(MOTORA_PWM, 0);
    softPwmWrite(MOTORB_PWM, 0);
}

void drive(const int left_cmd, const int right_cmd)
{
    ROS_INFO("Drive: Left = %i, Right = %i", left_cmd, right_cmd);

    if (right_cmd >= 0)
    {
        digitalWrite(MOTORA_FORWARD, LOW);
    }
    else
    {
        digitalWrite(MOTORA_FORWARD, HIGH);
    }
    if (left_cmd >= 0)
    {
        digitalWrite(MOTORB_FORWARD, LOW);
    }
    else
    {
        digitalWrite(MOTORB_FORWARD, HIGH);
    }
    softPwmWrite(MOTORA_PWM, std::abs(right_cmd));
    softPwmWrite(MOTORB_PWM, std::abs(left_cmd));

}

void turn(const Direction dir, const unsigned int speed)
{
    if (dir == CW)
    {
        ROS_INFO("Turning CW: Speed = %i", speed);
        digitalWrite(MOTORA_FORWARD, LOW);
        digitalWrite(MOTORB_FORWARD, HIGH);
    }
    else
    {
        ROS_INFO("Turning CCW: Speed = %i", speed);
        digitalWrite(MOTORA_FORWARD, HIGH);
        digitalWrite(MOTORB_FORWARD, LOW);
    }
    softPwmWrite(MOTORA_PWM, speed);
    softPwmWrite(MOTORB_PWM, speed);
}

void drivePI(int heading, float dt)
{
    int heading_error = last_command_msg.heading - heading;
    // Keep heading error centered at 0 between -180 and 180
    if (heading_error > 180)
    {
        heading_error -= 360;
    }
    else if (heading_error < -180)
    {
        heading_error += 360;
    }

    if (std::abs(heading_error_drive_sum + heading_error * dt) <= INT_CLAMP)
    {
        heading_error_drive_sum += heading_error * dt;
    }

    float heading_command = heading_error * KP_DRIVE + heading_error_drive_sum * KI_DRIVE;

    // Note: this PI calculation assumes forward motion, since the robot should never have to reverse
    // Except for construction check, but the errors will be 0 for said check

    // This calculation maps the desired speed between 0-100, then adds the speed PI correction and the heading
    // correction
    // Heading correction is subtracted for right wheel (positive heading command means clockwise turn)
    int right_speed = (int)((last_command_msg.speed / MAX_VEL) * PWM_RANGE + heading_command);
    int left_speed = (int)((last_command_msg.speed / MAX_VEL) * PWM_RANGE - heading_command);

    // Clamp speeds, bound checking but be sure to retain the ratio
    if (std::abs(right_speed) >= std::abs(left_speed))
    {
        float ratio = std::abs(float(right_speed) / left_speed);
        if (std::abs(right_speed) > PWM_RANGE)
        {
            right_speed = (int)(std::copysign(PWM_RANGE, right_speed));
            left_speed = (int)(std::copysign((right_speed / ratio), left_speed));  // Reduce other speed to match the other wheel
        }
    }
    else
    {
        float ratio = std::abs(float(left_speed) / right_speed);
        if (std::abs(left_speed) > PWM_RANGE)
        {
            left_speed = (int)(std::copysign(PWM_RANGE, left_speed));
            right_speed = (int)(std::copysign((left_speed / ratio), right_speed));  // Reduce other speed to match the other wheel
        }
    }
    drive(left_speed, right_speed);
}

void turningCallback(int heading, float dt)
{
    int heading_error = last_command_msg.heading - heading;

    // Keep heading error centered at 0 between -180 and 180
    if (heading_error > 180)
    {
        heading_error -= 360;
    }
    else if (heading_error < -180)
    {
        heading_error += 360;
    }

    if (std::abs(heading_error_turn_sum + heading_error * dt) <= INT_CLAMP)
    {
        heading_error_turn_sum += heading_error * dt;
    }

    int turning_speed = (int)std::abs(heading_error * KP_TURNING + heading_error_turn_sum * KI_TURNING);
    if (turning_speed > MAX_TURNING_SPEED)
        turning_speed = MAX_TURNING_SPEED;

    if (heading_error >= 0)
    {
        turn(CCW, turning_speed);
    }
    else
    {
        turn(CW, turning_speed);
    }
    // Could add a condition here to stop turning if within a certain threshold, right now
    // We leave that up to the planner
}

void positionCallback(const bill_msgs::Position::ConstPtr& msg)
{
    int heading = msg->heading;
    last_heading = heading;
    auto time_now = std::chrono::high_resolution_clock::now();

    if (last_command_msg.command == bill_msgs::MotorCommands::TURN)
    {
        turningCallback(heading, std::chrono::duration<float>(time_now - last_msg_time).count());
    }
    else if (last_command_msg.command == bill_msgs::MotorCommands::DRIVE)
    {
        drivePI(heading, std::chrono::duration<float>(time_now - last_msg_time).count());
    }

    last_msg_time = time_now;

}

void motorCallback(const bill_msgs::MotorCommands::ConstPtr& msg)
{
    last_command_msg.command = msg->command;
    last_command_msg.heading = msg->heading;
    last_command_msg.speed = msg->speed;

    if (msg->command == bill_msgs::MotorCommands::STOP)
    {
        ROS_INFO("Calling Stop");
        stop();
    }
    else if (msg->command == bill_msgs::MotorCommands::TURN)
    {
        ROS_INFO("Calling Turn");
        // Call these here to start a robots action, callback from odom continues the action until completion
        last_msg_time = std::chrono::high_resolution_clock::now();
        turningCallback(last_heading, 0);
    }
    else
    {
        ROS_INFO("Calling Drive");
        // Call these here to start a robots action, callback from odom continues the action until completion
        last_msg_time = std::chrono::high_resolution_clock::now();
        drivePI(last_heading, 0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver");
    wiringPiSetupGpio();
    pinMode(MOTORA_FORWARD, OUTPUT);
    pinMode(MOTORB_FORWARD, OUTPUT);
    pinMode(MOTORA_PWM, OUTPUT);
    pinMode(MOTORB_PWM, OUTPUT);
    softPwmCreate(MOTORA_PWM, 0, PWM_RANGE);
    softPwmCreate(MOTORB_PWM, 0, PWM_RANGE);

    ros::NodeHandle nh;
    ros::Subscriber sub_motor = nh.subscribe("motor_cmd", 1, motorCallback);
    ros::Subscriber sub_odom = nh.subscribe("position", 1, positionCallback);

    // Load parameters from yaml
    nh.getParam("/bill/motor_params/kp_turning", KP_TURNING);
    nh.getParam("/bill/motor_params/ki_turning", KI_TURNING);
    nh.getParam("/bill/motor_params/kp_drive", KP_DRIVE);
    nh.getParam("/bill/motor_params/ki_drive", KI_DRIVE);
    last_command_msg.command = bill_msgs::MotorCommands::STOP;

    ros::spin();
    return 0;
}
