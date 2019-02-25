#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "angles/angles.h"
#include "bill_msgs/MotorCommands.h"
#include "nav_msgs/Odometry.h"
#include "wiringPi.h"
#include <softPwm.h>
#include <mutex>

#define PWM_RANGE 100 // Max pwm value
#define TURNING_SPEED 30
#define INT_CLAMP 2
#define dt 0.1 // TODO: Define this based off the frequency the odom channel gets published
#define KP_HEADING 10
#define KI_HEADING 1
#define KP_SPEED 10
#define KI_SPEED 5
#define MAX_VEL 0.4
// Motor A is right motor
#define MOTORA_PWM 20 // PWMA
#define MOTORA_FORWARD 21 // AIN1
#define MOTORA_REVERSE 22 // AIN2
// Motor B is left motor
#define MOTORB_PWM 23 // PWMB
#define MOTORB_FORWARD 24 // BIN1
#define MOTORB_REVERSE 25 // BIN2

//std::mutex mutex;
bill_msgs::MotorCommands last_command_msg;
int last_heading = 0;
float last_speed = 0;
float speed_error_sum = 0;
float heading_error_sum = 0;

enum direction{
    CW = 1,
    CCW = -1
};

void stop()
{
    ROS_INFO("Stop");
    digitalWrite(MOTORA_FORWARD, LOW);
    digitalWrite(MOTORA_REVERSE, LOW);
    digitalWrite(MOTORB_FORWARD, LOW);
    digitalWrite(MOTORB_REVERSE, LOW);
    softPwmWrite(MOTORA_PWM, 0);
    softPwmWrite(MOTORB_PWM, 0);
}

void drive(const int left_cmd, const int right_cmd)
{
    ROS_INFO("Drive: Left = %i, Right = %i", left_cmd, right_cmd);

    if (right_cmd >= 0)
    {
        digitalWrite(MOTORA_FORWARD, HIGH);
        digitalWrite(MOTORA_REVERSE, LOW);
    }
    else
    {
        digitalWrite(MOTORA_FORWARD, LOW);
        digitalWrite(MOTORA_REVERSE, HIGH);
    }
    if (left_cmd >= 0)
    {
        digitalWrite(MOTORB_FORWARD, HIGH);
        digitalWrite(MOTORB_REVERSE, LOW);
    }
    else
    {
        digitalWrite(MOTORB_FORWARD, LOW);
        digitalWrite(MOTORB_REVERSE, HIGH);
    }
    softPwmWrite(MOTORA_PWM, std::abs(right_cmd));
    softPwmWrite(MOTORB_PWM, std::abs(left_cmd));
}

void turn(const int dir, const unsigned int speed)
{
    if (dir == direction::CW)
    {
        ROS_INFO("Turning CW: Speed = %i", speed);
        digitalWrite(MOTORA_FORWARD, LOW);
        digitalWrite(MOTORA_REVERSE, HIGH);
        digitalWrite(MOTORB_FORWARD, HIGH);
        digitalWrite(MOTORB_REVERSE, LOW);
    }
    else
    {
        ROS_INFO("Turning CCW: Speed = %i", speed);
        digitalWrite(MOTORA_FORWARD, HIGH);
        digitalWrite(MOTORA_REVERSE, LOW);
        digitalWrite(MOTORB_FORWARD, LOW);
        digitalWrite(MOTORB_REVERSE, HIGH);
    }
    softPwmWrite(MOTORA_PWM, speed);
    softPwmWrite(MOTORB_PWM, speed);
}

void drivePI(int heading, float speed)
{
    // NOTE: Writing speed PI for now, although open loop speed control may be good enough for our purposes
    //std::lock_guard<std::mutex> lk(mutex);
    // TODO: Remove 0 when odom measurements work
    int heading_error = 0; //last_command_msg.heading - heading;
    // Keep heading error centered at 0 between -180 and 180
    if (heading_error > 180)
    {
        heading_error -= 360;
    }
    else if (heading_error < -180)
    {
        heading_error +=360;
    }

    // TODO: Remove 0 when odom measurements work
    float speed_error = 0; //last_command_msg.speed - speed;
    if (std::abs(speed_error_sum + speed_error * dt) < INT_CLAMP)
    {
        speed_error_sum += speed_error * dt;
    }
    if (std::abs(heading_error_sum + heading_error * dt) < INT_CLAMP)
    {
        heading_error_sum += heading_error * dt;
    }
    int speed_command = speed_error * KP_SPEED + speed_error_sum * KI_SPEED;
    int heading_command = heading_error * KP_HEADING + heading_error_sum * KI_HEADING;

    // Note: this PI calculation assumes forward motion, since the robot should never have to reverse
    // Except for construction check, but the errors will be 0 for said check

    // This calculation maps the desired speed between 0-100, then adds the speed PI correction and the heading correction
    // Heading correction is subtracted for right wheel (positive heading command means clockwise turn)
    int right_speed = (last_command_msg.speed / float(MAX_VEL)) * PWM_RANGE + speed_command - heading_command;
    int left_speed = (last_command_msg.speed / float(MAX_VEL)) * PWM_RANGE + speed_command + heading_command;

    // Clamp speeds, bound checking
    if (right_speed > PWM_RANGE)
    {
        right_speed = PWM_RANGE;
    }
    else if (right_speed < -1 * PWM_RANGE)
    {
        right_speed = -1 * PWM_RANGE;
    }
    if (left_speed > PWM_RANGE)
    {
        left_speed = PWM_RANGE;
    }
    else if (left_speed < -1 * PWM_RANGE)
    {
        left_speed = -1 * PWM_RANGE;
    }

    drive(left_speed, right_speed);

}

void turningCallback(int heading)
{
    ROS_INFO("In Turning Callback");
    //std::lock_guard<std::mutex> lk(mutex);
    int error = last_command_msg.heading - heading;
    if (error >= 0 && error <= 180 || error < 0 && error >= -180)
    {
        // For now turn at a constant speed always
        turn(direction::CW, TURNING_SPEED);
    }
    else
    {
        // For now turn at a constant speed always
        turn(direction::CCW, TURNING_SPEED);
    }
    // Could add a condition here to stop turning if within a certain threshold, right now
    // We leave that up to the planner
}

void fusedOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // TODO: Pull out heading as angle, (speed?) as PID input
    //std::lock_guard<std::mutex> lk(mutex);
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    float heading = angles::to_degrees(tf::getYaw(pose.getRotation()));
    float speed = msg->twist.twist.linear.x; //
    last_heading = heading;
    last_speed = speed;

    if (last_command_msg.command == bill_msgs::MotorCommands::TURN)
    {
        turningCallback(heading);
    }
    else if (last_command_msg.command == bill_msgs::MotorCommands::DRIVE)
    {
        drivePI(heading, speed);
    }
}

void motorCallback(const bill_msgs::MotorCommands::ConstPtr& msg)
{
    //std::lock_guard<std::mutex> lk(mutex);
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
        turningCallback(last_heading);
    }
    else
    {
        ROS_INFO("Calling Drive");
        // Call these here to start a robots action, callback from odom continues the action until completion
        drivePI(last_heading, last_speed);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_driver");
    wiringPiSetupGpio();
    pinMode(MOTORA_FORWARD, OUTPUT);
    pinMode(MOTORA_REVERSE, OUTPUT);
    pinMode(MOTORB_FORWARD, OUTPUT);
    pinMode(MOTORB_REVERSE, OUTPUT);
    softPwmCreate(MOTORA_PWM, 0, PWM_RANGE);
    softPwmCreate(MOTORB_PWM, 0, PWM_RANGE);
    ros::NodeHandle nh;
    ros::Subscriber sub_motor = nh.subscribe("motor_cmd", 1, motorCallback);
    ros::Subscriber sub_odom = nh.subscribe("fused_odometry", 1, fusedOdometryCallback);

    ros::spin();
    return 0;
}
