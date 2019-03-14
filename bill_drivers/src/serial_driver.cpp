#include "bill_msgs/Survivor.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "wiringPi.h"
#include "serial/serial.h"
#include "bill_drivers/constant_definition.hpp"
#include <bitset>
#include <stdexcept>

const std::string port = "/dev/ttyACM0";
const int baud = 9600;
const std::string delimiter = " ";
const int msg_length = 13;

char sensorData[msg_length];

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_driver");
    ros::NodeHandle nh;
    ros::Rate loop_rate(LOOP_RATE_SERIAL);

    ros::Publisher survivor_pub = nh.advertise<bill_msgs::Survivor>("survivors", 100);
    ros::Publisher food_pub = nh.advertise<std_msgs::Bool>("food", 100);
    ros::Publisher fire_pub = nh.advertise<std_msgs::Bool>("fire", 100);
    ros::Publisher fire_left_pub = nh.advertise<std_msgs::Bool>("fire_left", 100);
    ros::Publisher fire_right_pub = nh.advertise<std_msgs::Bool>("fire_right", 100);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

    std::bitset<8> fire_bitmap = 0x01;
    std::bitset<8> food_bitmap = 0x02;
    std::bitset<8> survivor_bitmap = 0x0C;
    std::bitset<8> fire_left_bitmap = 0x10;
    std::bitset<8> fire_right_bitmap = 0x20;

    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    if (my_serial.isOpen())
    {
        ROS_INFO("Serial port is open!");
    }
    else
    {
        ROS_ERROR("Serial port not open!!!");
    }

    while (ros::ok())
    {
        bill_msgs::Survivor survivor_msg;
        std_msgs::Bool food_msg;
        std_msgs::Bool fire_msg;
        std_msgs::Bool fire_left_msg;
        std_msgs::Bool fire_right_msg;

        std::string data = my_serial.readline(65536, "\r\n");
        size_t pos = 0;
        int i = 0;
        std::string token;
        while ((pos = data.find(delimiter)) != std::string::npos)
        {
            if (i >= msg_length)
            {
                i++;  // Increment i so the check later recognizes the message as invalid
                break;
            }
            token = data.substr(0, pos);
            try
            {
                sensorData[i] = (char)std::stoi(token);
            }
            catch (const std::invalid_argument& ia)
            {
                ROS_ERROR("Read a broken imu string: %s, bailing this read", ia.what());
                break;
            }
            data.erase(0, pos + delimiter.length());
            i++;
        }

        // Unless message read was the appropriate length, then toss it
        if (i == msg_length)
        {
            // Convert IMU Quaternion data
            double quatW = (double)((signed short)(sensorData[1] << 8 | sensorData[0])) / 16384.0;
            double quatX = (double)((signed short)(sensorData[3] << 8 | sensorData[2])) / 16384.0;
            double quatY = (double)((signed short)(sensorData[5] << 8 | sensorData[4])) / 16384.0;
            double quatZ = (double)((signed short)(sensorData[7] << 8 | sensorData[6])) / 16384.0;

            // Convert linear acceleration and angular velocity
            double xaccel = (double)(((signed short)(sensorData[9] << 8 | sensorData[8])) / 100.0);
            double zgyro = (double)(((signed short)(sensorData[11] << 8 | sensorData[10])) / 16.0);

            sensor_msgs::Imu imu_msg;
            imu_msg.orientation.x = quatX;
            imu_msg.orientation.y = quatY;
            imu_msg.orientation.z = quatZ;
            imu_msg.orientation.w = quatW;

            imu_msg.angular_velocity.z = zgyro;
            imu_msg.linear_acceleration.x = xaccel;

            // Print out what the Arduino is sending...
            std::bitset<8> data_received(sensorData[12]);

            // ROS_INFO("Received arduino data: %i", (int)data_received.to_ulong());
            // Parse the byte, bits 3 and 2 are survivor data, 1 is food and 0 is fire
            survivor_msg.data = (int)((data_received & survivor_bitmap) >> 2).to_ulong();
            food_msg.data = (bool)(data_received & food_bitmap).to_ulong();
            fire_msg.data = (bool)(data_received & fire_bitmap).to_ulong();
            fire_left_msg.data = (bool)(data_received & fire_left_bitmap).to_ulong();
            fire_right_msg.data = (bool)(data_received & fire_right_bitmap).to_ulong();

            // Publish message, and spin thread
            imu_pub.publish(imu_msg);
            survivor_pub.publish(survivor_msg);
            food_pub.publish(food_msg);
            fire_pub.publish(fire_msg);
            fire_left_pub.publish(fire_left_msg);
            fire_left_pub.publish(fire_right_msg);
            ros::spinOnce();
        }
        loop_rate.sleep();
    }
    return 0;
}
