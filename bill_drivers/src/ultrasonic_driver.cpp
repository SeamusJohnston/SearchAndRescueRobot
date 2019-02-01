#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "wiringPi.h"
#include <iostream>
#include <limits> 		// for NaN

// Pin 23 is Echo, pin 24 is trigger
#define TRIGGER_PIN 24
#define ECHO_PIN 25
const static int ECHO_RECIEVE_TIMEOUT = 30000;
const static int ECHO_READ_TIMEOUT = 30000;
const static float DISTANCE_SCALE_CM = 58.0;
// Assumes speed of sound is 340 m/s
// The below value must be converted to cm/microsecond
const static float SPEED_OF_SOUND_CMPUS = 0.0340;

void setup()
{
    // Init GPIO 
    wiringPiSetupGpio();

    // Set up pins 
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    ROS_INFO("Trigger pin is: %i", wpiPinToGpio(TRIGGER_PIN));
    ROS_INFO("Echo pin is: %i", wpiPinToGpio(ECHO_PIN));

    // Init trigger as low
    digitalWrite(TRIGGER_PIN, LOW);
}

float ReadDistance()
{
    // Make sure the trigger pin starts low
    digitalWrite(TRIGGER_PIN,LOW);
    delayMicroseconds(2);

    // Send pulse of 10 microseconds
    digitalWrite(TRIGGER_PIN, HIGH);
    // Note this delay MUST be longer than 10 microseconds
    delayMicroseconds(20);
    digitalWrite(TRIGGER_PIN, LOW);

    // Wait for echo to return 
    int recieveTimeout = ECHO_RECIEVE_TIMEOUT;

    while (digitalRead(ECHO_PIN) == LOW)
    {
        if (--recieveTimeout == 0)
        {
            ROS_WARN("Ultrasonic sensor never receieved echo");
            return std::numeric_limits<double>::quiet_NaN();
        }
    }
  
  
    // Echo has been detected, measure how long its high for
    // *High by Young Thug and Elton John plays softly in the background*

    int readTimeout = ECHO_READ_TIMEOUT;
    long startTime = micros();

    while (digitalRead(ECHO_PIN) == HIGH)
    {
        if (--readTimeout == 0)
        {
            ROS_WARN("Ultrasonic sensor timed out while reading echo %i", micros()- startTime);
            return std::numeric_limits<double>::quiet_NaN();
        }
    }
    
    long echoTime = micros() - startTime;
    
    return(echoTime * SPEED_OF_SOUND_CMPUS)/ 2.0;
}

std_msgs::Float32 read()
{
    float distance = ReadDistance();

    if (std::isnan(distance))
    {
        ROS_WARN("Error on ultrasonic sensor!");
    }

    // Create message to publish
    std_msgs::Float32 msg;
    // Populate the message
    msg.data = distance;

    std::cout << "Ultrasonic sensor distance: " << distance << " cm"<< std::endl;
    return msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasonic_driver");
    ros::NodeHandle nh;
    ros::Publisher ultrasonic_pub = nh.advertise<std_msgs::Float32>("ultrasonic", 100);
    ros::Rate loop_rate(10);

    // Call sensor setup
    setup();

    while (ros::ok())
    {
        std_msgs::Float32 msg = read();

        // Publish message, and spin thread
        ultrasonic_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
