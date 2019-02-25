#include <Wire.h>

// Digital
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define Colour 3 // 3 OR 8 Digital

// Analog
#define Flame A0
#define Hall A1

// Globally Store Detected Colours
int colours[4];

void setup() 
{
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  pinMode(Colour, INPUT);
  pinMode(Flame, INPUT);
  pinMode(Hall, INPUT);
  
  // Setting frequency-scaling
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  Serial.begin(9600);
  Wire.begin(0x07); //Set Arduino up as an I2C slave at address 0x07
  Wire.onRequest(requestEvent); //Prepare to send data
  Wire.onReceive(receiveEvent); //Prepare to recieve data
}

void loop() 
{
}

void requestEvent()
{
  byte message = readColourSensor() ^ readFlameSensor() ^ readHallEffectSensor();
  
  Serial.println(message);
  Wire.write(message);
}

void receiveEvent(int numBytes)
{
  //Set Up Vars
  int receive_int=0;
  int count=0;

  //We'll recieve one byte at a time. Stop when none left
  while(Wire.available())
  {
    char c = Wire.read(); // receive a byte as character
    //Create Int from the Byte Array
    receive_int = c << (8 * count) | receive_int;
    count++;
  }
  Serial.print("Received Number: ");
  Serial.println(receive_int);
}

byte readFlameSensor()
{
  int sensorValue = analogRead(Flame);
  
  Serial.print("Flame: ");
  Serial.print(sensorValue);
  if(sensorValue < 70)
  {
    Serial.println(" True");
    return 0x01;
  }
  else
  {
    Serial.println(" False");
    return 0x00;
  }
}

byte readHallEffectSensor()
{
  int sensorValue = analogRead(Hall);
  
  Serial.print("Hall: ");
    Serial.print(sensorValue);
  if(sensorValue <= 510 || sensorValue >= 540)
  {
    Serial.println(" True");
    return 0x02;
  }
  else
  {
    Serial.println(" False");
    return 0x00;
  }
}

byte readColourSensor()
{
  ReadRGBC();
  
  if (LargeBuilding())
  {
    Serial.println("Detected Small Building");
    return 0x08;
  }
  else if (SmallBuilding())
  {
    Serial.println("Detected Large Building");
    return 0x04;
  }
}

void ReadRGBC()
{
  // RED
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  delay(100);
  colours[0] = pulseIn(Colour, LOW);
  
  // Green
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  delay(100);
  colours[1] = pulseIn(Colour, LOW);

  // Blue 
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  delay(100);
  colours[2] = pulseIn(Colour, LOW);
  
  // Clear
  digitalWrite(S2,HIGH);
  digitalWrite(S3,LOW);
  delay(100);
  colours[3] = pulseIn(Colour, LOW);

  Serial.print("rgbc: ");
  Serial.print(colours[0]);
  Serial.print(", ");
  Serial.print(colours[1]);
  Serial.print(", ");
  Serial.print(colours[2]);
  Serial.print(", ");
  Serial.println(colours[3]);
}

//110, 215, 209, 64
bool LargeBuilding()
{
  return colours[3] < 65 && // Clear Building Detected
    colours[0] > 80 && colours[0] < 120 && // Red in range
    colours[1] > 180 && colours[1] < 230 && // Blue in range
    colours[2] > 150 && colours[2] < 200; // Green in range
}

bool SmallBuilding()
{
  return colours[3] < 60 && // Clear Building Detected
    colours[0] > 30 && colours[0] < 80 && // Red in range
    colours[1] > 70 && colours[1] < 130 && // Blue in range
    colours[2] > 130 && colours[2] < 170; // Green in range
}
