#ifndef CONSTDEF
#define CONSTDEF

// Pins
const int LED_PIN = 19;
const int BUTTON_PIN = 6;
const int RELAY_OUTPIN = 26;

// Motors
const int MOTORA_PWM = 12;      // PWMA
const int MOTORB_PWM = 10;      // PWMB
const int MOTORA_FORWARD = 13;  // MA
const int MOTORB_FORWARD = 23;  // MB

// Encoders
const int MOTORB_ENCA_PIN = 7;   // Motor B (left) A channel
const int MOTORA_ENCA_PIN = 25;  // Motor A (right) A channel

// Constants
const float LOOP_RATE_ULTRA = 16.6667;
const float LOOP_RATE_ENCODER = 15;
const float LOOP_RATE_SERIAL = 100;
const float LOOP_RATE_RESET = 10;
const float LOOP_RATE_MAGNET = 10;

#endif
