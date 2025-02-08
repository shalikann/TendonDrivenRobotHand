#include <SoftwareSerial.h>
#include "RoboClaw.h"

// MOTOR SET UP
// Set up SoftwareSerial on pins 10 (RX) and 11 (TX)
SoftwareSerial serial(10, 11);  
RoboClaw roboclaw(&serial, 10000); // RoboClaw object
#define address 0x80 // Define the RoboClaw address

// Velocity PID, set to zero for position PID
#define Kp 2
#define Ki 0.2
#define Kd 0
#define qpps 7875

//Position PID
#define PosKp 2000
#define PosKi 0
#define PosKd 50000
#define KiMax 0
#define DeadZone 10
#define Min -10000
#define Max 10000

// ENCODER SET UP
volatile int counter = 0; // increase or decrease depending on the rotation of encoder
float angle = 0;
float linear_distance = 0;
const int cpr = 4000;
const float radius = 5.75; // mm, radius of encoder capstan


void setup() {
  // Initialize serial communication
  Serial.begin(9600);   // For Serial Monitor
  roboclaw.begin(38400);  // For RoboClaw communication


  roboclaw.SetM1VelocityPID(address,Kp,Ki,Kd,qpps);
//  roboclaw.SetM1PositionPID(address,PosKp,PosKi,PosKd,KiMax,DeadZone,Min,Max);
  
  // Encoder -- A leads B is clockwise, B leads A in counterclockwise
   pinMode(2, INPUT_PULLUP); // internal pullup input pin 2, A (brown)
   pinMode(3, INPUT_PULLUP); // internal pullup input pin 3, B (red)
   pinMode(18, INPUT_PULLUP);

  //Arduino Mega interrupt pins: 2, 3, 18, 19, 20, 21
   attachInterrupt(digitalPinToInterrupt(2), A, RISING);
   attachInterrupt(digitalPinToInterrupt(3), B, RISING);
   attachInterrupt(digitalPinToInterrupt(18), zero, RISING);
   
  // Nothing happens until encoder is initialized at zero
//   while(digitalRead(18)==LOW){
//   } 
//   counter = 0;
}


void loop() {
  moveMotor();
}


void moveMotor(void) {
//  roboclaw.SpeedAccelDeccelPositionM1(address, 0, qpps, 0, 6000, 1);
//  roboclaw.SpeedAccelDeccelPositionM1(address, 0, qpps, 0, 3000, 0);

  uint8_t depth1,depth2;
  roboclaw.SpeedDistanceM1(address,2000,600,1);
  while(true){
    roboclaw.ReadBuffers(address,depth1, depth2);
    readEncoder();
//    readMotorEncoder();
    if (depth1 == 0x80) break;
  }
  roboclaw.SpeedDistanceM1(address,-2000,600,1);
  while(true){
    roboclaw.ReadBuffers(address,depth1,depth2);
    readEncoder();
//    readMotorEncoder();
    if (depth1==0x80) break;
  }

}

void readMotorSpeed(void) {
  uint8_t status;
  bool valid;
  int32_t speed = roboclaw.ReadSpeedM1(address, &status, &valid);
  Serial.print("Actual speed:");
  Serial.print(speed);
  Serial.print("\n");
}

void readMotorEncoder(void) {
  uint8_t status; // 80 (increase) or 82 (decrease) 
  bool valid;
  int32_t enc= roboclaw.ReadEncM1(address, &status, &valid);
  Serial.print("MotorEncoder:");
  Serial.print(enc,DEC);
  Serial.print("\n");
}


void readEncoder(void) {
//   360 degree at bottom, resets to zero there
    angle = (counter * 360.0) / cpr; // Angle in degrees
//    linear_distance = (angle * PI / 180.0) * radius; // Linear distance in mm
    Serial.print("counter:");
    Serial.print(counter);
    Serial.print("\n");
//    Serial.print("angle:");
//    Serial.print(angle); // Two decimal precision
//    Serial.print("\n");
}

void A() {
  // A is activated if DigitalPin 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter++;
  }else{
    counter--;
  }
}
   
void B() {
  // B is activated if DigitalPin 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter--;
  }else{
    counter++;
  }
}

void zero() {
  if(digitalRead(18)==HIGH) { // zeros every 180 degrees
    counter=0;
  }
}
