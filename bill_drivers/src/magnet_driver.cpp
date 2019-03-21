#include "bill_drivers/MPU9250_Passthru.h"
#include "ros/ros.h"
#include "bill_drivers/constant_definition.hpp"
#include "std_msgs/Bool.h"
#include <wiringPi.h>
#include <stdio.h>

static const MPUIMU::Gscale_t  GSCALE    = MPUIMU::GFS_250DPS;
static const MPUIMU::Ascale_t  ASCALE    = MPUIMU::AFS_2G;
static const MPU9250::Mscale_t MSCALE    = MPU9250::MFS_16BITS;
static const MPU9250::Mmode_t  MMODE     = MPU9250::M_100Hz;
static const uint8_t SAMPLE_RATE_DIVISOR = 0x04;

static MPU9250_Passthru imu(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);

bool prev_magnet_state = false;
bool magnet_state = false;

void setup()
{
    // Setup WiringPi
    wiringPiSetupGpio();
    delay(100);

    // Start the MPU9250
    switch (imu.begin()) {

        case MPUIMU::ERROR_IMU_ID:
            ROS_ERROR("Bad IMU device ID");
            break;
        case MPUIMU::ERROR_MAG_ID:
            ROS_ERROR("Bad magnetometer device ID");
            break;
        case MPUIMU::ERROR_SELFTEST:
            //errmsg("Failed self-test");
            break;
        default:
            ROS_INFO("MPU9250 online!\n");
            break;
    }

    // Comment out if using pre-measured, pre-stored offset magnetometer biases
    //printf("Mag Calibration: Wave device in a figure eight until done!\n");
    //imu.calibrateMagnetometer();
}

std_msgs::Bool isMagnetPresent() {
    // Detect magnet
    std_msgs::Bool msg;
    msg.data = false;
    float mx, my, mz;
    if (imu.checkNewMagData()) {
        imu.readMagnetometer(mx, my, mz);
        ROS_INFO("New Mag Data: X: %f Y: %f Z: %f", mx, my, mz);

        // TODO: Add threshold and logic for returning true here
    }
    magnet_state = msg.data;
    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magnet_driver");
    ros::NodeHandle nh;
    ros::Publisher food_pub = nh.advertise<std_msgs::Bool>("food", 100, true);
    ros::Rate loop_rate(LOOP_RATE_MAGNET);

    // Call sensor setup
    setup();

    while (ros::ok())
    {
        std_msgs::Bool msg = isMagnetPresent();
        if (magnet_state != prev_magnet_state)
        {
            //  Only publish a state change
            food_pub.publish(msg);
            ros::spinOnce();
            prev_magnet_state = magnet_state;
        }
        loop_rate.sleep();
    }
    return 0;
}
