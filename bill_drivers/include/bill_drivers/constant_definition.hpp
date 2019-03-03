#ifndef CONSTDEF
#define CONSTDEF

// Pins
const int LED_PIN = 19;
const int BUTTON_PIN = 26;
const int RELAY_OUTPIN = 6;

// Ultrasonic
const int ULTRA_TRIGGER_PIN1 = 17;
const int ULTRA_ECHO_PIN1 = 27;
const int ULTRA_TRIGGER_PIN2 = 9;
const int ULTRA_ECHO_PIN2 = 11;

// Motors
const int MOTORA_PWM = 12; // PWMA
const int MOTORB_PWM = 13;// PWMB
const int MOTORA_FORWARD = 10; // AIN1
const int MOTORA_REVERSE = 22; // AIN2
const int MOTORB_FORWARD = 23; // BIN1
const int MOTORB_REVERSE = 24; // BIN2

// Encoders
const int MOTORB_ENCB_PIN = 1; // Motor B (left) B channel
const int MOTORB_ENCA_PIN = 7; // Motor B (left) A channel
const int MOTORA_ENCB_PIN = 8; // Motor A (right) B channel
const int MOTORA_ENCA_PIN = 25; // Motor A (right) A channel

//Constants
const float LOOP_RATE_ULTRA = 16.6667;
const float LOOP_RATE_ENCODER = 10;
const float LOOP_RATE_SERIAL = 100;
const float LOOP_RATE_RESET = 10;

#endif
