#include <Wire.h>
#include <NewPing.h>
#include <TimerOne.h>

// Digital
/*
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define Colour 3
*/
// Analog
#define Flame A0
#define Hall A1

// Motor
#define DIR_A 12
#define DIR_B 13
#define BRAKE_A 9
#define BRAKE_B 8
#define PWM_A 3
#define PWM_B 11

// Led 
#define LED 10

// Sonar
#define TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// IMU Registers 
#define PAGE_SWAP     0x07
#define ACC_CONF      0x08
#define GYR_CONF_0    0x0A
#define GYR_CONF_1    0x0B
#define MAG_CONF      0x09
#define TEMP_SOURCE   0x40
#define UNIT_SEL      0x3B
#define PWR_MODE      0x3E

#define ADDRESS       0x28
#define HEADING       0x1A
#define ORIENTATION   0x20
#define XACCEL        0x28
#define ZGYRO         0x18
#define MODE_REG      0x3D

#define FUSION_MODE   0x0C // NDOF?

// Globally Store Detected Colours
int colours[4];
int count = 0;
bool turning = false;

// Global byte array for all of our IMU data
// 8 for LSB and MSB of Orientation Quaternion x,y,z and w
// 2 for LSB and MSB of Linear Acceleration in x
// 2 for LSB And MSB of Gyro in Z
byte data[13];

void setup() 
{
  pinMode(Flame, INPUT);
  pinMode(Hall, INPUT);

  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  Serial.begin(9600);
  //Timer1.initialize(5000000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  //Timer1.attachInterrupt( plan ); // attach the service routine here
  //analogWrite(PWM_A, 0);
  //analogWrite(PWM_B, 0);
  plan();
}
  //Wire.begin(0x07); //Set Arduino up as an I2C slave at address 0x07
  //Wire.onRequest(requestEvent); //Prepare to send data
  //Wire.onReceive(receiveEvent); //Prepare to recieve data

  // Config IMU 
  //delay(20);
  //config_BNO055();
  //delay(20);
  


void loop()
{
  /*
    bool fire = readFlameSensor();
    if (fire)
      digitalWrite(LED, HIGH);
    else
      digitalWrite(LED, LOW);
      
    int dist = sonar.ping_cm();
    Serial.print("Distance: ");
    Serial.println(dist);
    if (dist <= 30 && dist != 0)
    {
      Serial.println("Turning");
      turning = true;
      Timer1.stop();
      digitalWrite(DIR_A, LOW);
      digitalWrite(DIR_B, HIGH);
      analogWrite(PWM_A, 255);
      analogWrite(PWM_B, 255);
    }
    else if (turning)
    {
      Serial.println("Stop turning");
      turning = false;
      count--;
      if (count < 0)
        count == 3;
      plan();
      Timer1.resume();
    }
    */
    
}

void plan()
{
  if (count == 0)
  {
    Serial.println("Forward");
    digitalWrite(DIR_A, LOW);
    digitalWrite(DIR_B, LOW);
    analogWrite(PWM_A, 255);
    analogWrite(PWM_B, 255);
  }
  if (count% 2 == 1)
  {
    Serial.println("Stop");
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
  }
  if (count == 2)
  {
    Serial.println("Backwards");
    digitalWrite(DIR_A, HIGH);
    digitalWrite(DIR_B, HIGH);
    analogWrite(PWM_A, 255);
    analogWrite(PWM_B, 255);
  }
  count = (count + 1) % 4;
}

bool readFlameSensor()
{
  int sensorValue = analogRead(Flame);
  
  //Serial.print("Flame: ");
  //Serial.print(sensorValue);
  if(sensorValue < 70)
  {
    //Serial.println(" True");
    return true;
  }
  else
  {
    //Serial.println(" False");
    return false;
  }
}

byte readHallEffectSensor()
{
  int sensorValue = analogRead(Hall);
  
  //Serial.print("Hall: ");
  //Serial.print(sensorValue);
  if(sensorValue <= 510 || sensorValue >= 540)
  {
    //Serial.println(" True");
    return 0x02;
  }
  else
  {
    //Serial.println(" False");
    return 0x00;
  }
}

bool LargeBuilding()
{
  bool returnVal = colours[3] < 65 && // Clear Building Detected
    colours[0] > 50 && colours[0] < 100 && // Red in range
    colours[1] > 125 && colours[1] < 175 && // Blue in range
    colours[2] > 100 && colours[2] < 150; // Green in range
  return returnVal;
}

bool SmallBuilding()
{
  bool returnVal = colours[3] < 60 && // Clear Building Detected
    colours[0] > 30 && colours[0] < 80 && // Red in range
    colours[1] > 50 && colours[1] < 100 && // Blue in range
    colours[2] > 85 && colours[2] < 135;
  return returnVal;
}

/////////////////////
//                 //
//    IMU Code     //
//                 //
/////////////////////

void config_BNO055()
{
   Wire.beginTransmission(ADDRESS);
   Wire.write(PAGE_SWAP);                // Page swap
   Wire.write(1);                        // To page 1
   Wire.endTransmission();
    
   Wire.beginTransmission(ADDRESS);
   Wire.write(ACC_CONF);                // ACC Configuration
   Wire.write(0x08);                    // 0x08 acc conf| normal power, 31.25hz bandwidth, 2G range
   Wire.endTransmission();

   Wire.beginTransmission(ADDRESS);
   Wire.write(GYR_CONF_0);                // GYR Configuration
   Wire.write(0x23);                      // 0x32 250DPS scale, 23hz bandwidth
   Wire.endTransmission();

   Wire.beginTransmission(ADDRESS);
   Wire.write(GYR_CONF_1);                // GYR Configuration
   Wire.write(0x00);                      // Normal power
   Wire.endTransmission();
   
   Wire.beginTransmission(ADDRESS);
   Wire.write(MAG_CONF);                  // MAG Configuration
   Wire.write(0x1B);                      // Normal power, High accuracy, 10Hz output rate
   Wire.endTransmission();
   
   Wire.beginTransmission(ADDRESS);
   Wire.write(PAGE_SWAP);                // Page swap
   Wire.write(0);                        // To page 0
   Wire.endTransmission();
   
   Wire.beginTransmission(ADDRESS);
   Wire.write(TEMP_SOURCE);               // Temperature source
   Wire.write(0x01);                      // gyro
   Wire.endTransmission();
   
   Wire.beginTransmission(ADDRESS);
   Wire.write(UNIT_SEL);                  // Unit select
   Wire.write(0x01);                      // Temp in Degrees C, Heading in Degrees, Angular rate in DPS, Gravity in mg
   Wire.endTransmission();
   
   Wire.beginTransmission(ADDRESS);
   Wire.write(PWR_MODE);                  // Power mode normal
   Wire.write(0x00);                      // normal
   Wire.endTransmission();
   
   Wire.beginTransmission(ADDRESS);
   Wire.write(MODE_REG);                  // Operational mode NDOF
   Wire.write(FUSION_MODE);
   Wire.endTransmission();
}
