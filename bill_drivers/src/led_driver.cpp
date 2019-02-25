#include "ros/ros.h"
#include "wiringPi.h"
#include "std_msgs/Bool.h"

#define LED_PIN 18

bool debugLEDToggle = true;

void setup() {
	// Init GPIO 
    wiringPiSetupGpio();

    // Set up pins 
    pinMode(LED_PIN, OUTPUT);
	ROS_INFO("LED pin is: %i", LED_PIN);
}

void ledCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        // Turn on led
		digitalWrite(LED_PIN, HIGH);
    }
    else
    {
        // Turn off led
		digitalWrite(LED_PIN, LOW);
    }
}

int main(int argc, char** argv)
{
	// Set up LED
	setup();
	
    ros::init(argc, argv, "led_driver");
    ros::NodeHandle nh;
    ros::Subscriber sub_led = nh.subscribe("led", 1, ledCallback);

	// // For testing purposes	
	// while (ros::ok())
    // {
		// if (debugLEDToggle) {
			// digitalWrite(LED_PIN, HIGH);
		// } else {
			// digitalWrite(LED_PIN, LOW);
		// }
	
		// // Toggles flag
		// debugLEDToggle ^= true;
		// delay(1000);
	// }
	
    ros::spin();
    return 0;
}
