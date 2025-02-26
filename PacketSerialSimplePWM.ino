#include <RoboClaw.h>
enum State {
    WAITING,
    TENSIONING1,
    TENSIONING2,
    INITIALIZING,
    ZEROING,
    LIFTING,
    LOWERING
};

State currentState = WAITING;

// MOTOR SET UP
// Set up SoftwareSerial on pins 10 (RX) and 11 (TX)
SoftwareSerial serial(10, 11);  
RoboClaw roboclaw(&serial, 10000); // RoboClaw object
#define address 0x80 // Define the RoboClaw address

// Velocity PID for motor 1, set to zero for position PID
#define Kp1 2.2460
#define Ki1 0.2828
#define Kd1 0
#define qpps1 9000
// Velocity PID for motor 2
#define Kp2 2.3541
#define Ki2 0.2877
#define Kd2 0
#define qpps2 9562

//Position PID for motor 1
#define PosKp1 2000
#define PosKi1 0
#define PosKd1 50000
#define KiMax1 0
#define DeadZone1 10
#define Min1 -10000
#define Max1 10000

const int cpr_motor = 4752;

// ENCODER SET UP
volatile int counter = 0; // increase or decrease depending on the rotation of encoder
float angle = 0;
const int cpr = 4000;
const float radius = 5.75; // mm, radius of encoder capstan
volatile int zeroed = 0;
float delta_angle = 0;
int dist = 0;

void setup() {
  Serial.begin(9600);   // For Serial Monitor
  roboclaw.begin(38400);  // For RoboClaw communication

  roboclaw.SetM1VelocityPID(address,Kp1,Ki1,Kd1,qpps1);
  roboclaw.SetM2VelocityPID(address,Kp2,Ki2,Kd2,qpps2);
//  roboclaw.SetM1PositionPID(address,PosKp,PosKi,PosKd,KiMax,DeadZone,Min,Max);
  
  // Encoder -- A leads B is clockwise, B leads A in counterclockwise
   pinMode(2, INPUT_PULLUP); // internal pullup input pin 2, A (brown)
   pinMode(3, INPUT_PULLUP); // internal pullup input pin 3, B (red)
   pinMode(18, INPUT_PULLUP);

  //Arduino Mega interrupt pins: 2, 3, 18, 19, 20, 21
   attachInterrupt(digitalPinToInterrupt(2), A, RISING);
   attachInterrupt(digitalPinToInterrupt(3), B, RISING);
   attachInterrupt(digitalPinToInterrupt(18), encoderZero, RISING);
   
   Serial.println("Enter command: i (init), t (tension), s (start test), x (exit)");
}

void loop() {
    // Check for serial input
    if (Serial.available() > 0) {
        char command = Serial.read();
        switch (command) {
            case 'i':
                currentState = INITIALIZING; // initializing is started after zeroing is completed
                break;
            case 'z':
                currentState = ZEROING;
                break;
            case 't':
                currentState = TENSIONING1;
                break;
            case 'T':
                currentState = TENSIONING2;
                break;
            case 's':
                currentState = LIFTING;
                break;
            case 'x':
                currentState = WAITING;
                stopMotors();
                break;
            default:
                Serial.println("Invalid command.");
                break;
        }
    }

    // State Machine
    switch (currentState) {
        case WAITING:
            // No action in waiting state -- stop both motors
            stopMotors();
            break;

        case TENSIONING1:
            Serial.println("Tensioning...");
            displayEncoderAngle();
            moveMotor(1, 500, -500); // rotate M1 CW by 300 steps
            currentState = WAITING;
            break;

        case TENSIONING2:
            Serial.println("Tensioning...");
            displayEncoderAngle();
            moveMotor(2, 500, -500); // rotate M2 CCW by 300 steps
            currentState = WAITING;
            break;

        case ZEROING: // manually move until past zero mark
            Serial.println("Zeroing...");
            moveMotor(12, 500, 300);
            displayEncoderAngle();
            currentState = WAITING;
            break;

        case INITIALIZING: // move to 90 degrees
            Serial.println("Initializing...");
            
            delta_angle = 90 - counter;
            dist = delta_angle * cpr_motor / 360;
            Serial.println(dist);
            moveMotor(12, dist, -1000);
            displayEncoderAngle();
            currentState = WAITING;
            break;
        
        case LIFTING:
            Serial.println("Lifting...");
            moveMotor(12, 1000, -500);
            currentState = LOWERING;
            break;

        case LOWERING:
            Serial.println("Lowering...");
            moveMotor(12, 1000, 500);            
            currentState = LIFTING;
            break;

        default:
            break;
    }
}

// MOTOR FUNCTIONS

// Function to stop motors
void stopMotors() {
  roboclaw.ForwardM1(address, 0);
  roboclaw.ForwardM2(address, 0);
}

// move (M1: CW = neg speed, M2: CW = pos speed)
void moveMotor(int motor, int distance, int motor_speed) {
  uint8_t depth1,depth2;  
  if (motor == 1){
    roboclaw.SpeedDistanceM1(address,motor_speed,distance,1);
    while(true){
      roboclaw.ReadBuffers(address,depth1, depth2);
      Serial.print("\n");
      if (depth1 == 0x80) break;
    } 
  }
  if (motor == 2) {
    roboclaw.SpeedDistanceM2(address,motor_speed,distance,1);
    while(true){
      roboclaw.ReadBuffers(address,depth1, depth2);
      Serial.print("\n");
      if (depth2 == 0x80) break;
    } 
  }
  if (motor == 12) {
    roboclaw.SpeedDistanceM1M2(address, motor_speed, distance, -motor_speed, distance, 1);
    while(true){
      roboclaw.ReadBuffers(address,depth1,depth2);
      Serial.print("\n");
      if (depth1 == 0x80 && depth2 == 0x80) break;
    }
  }
}

// ENCODER FUNCTIONS
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

void encoderZero() {
  if(digitalRead(18)==HIGH) { // zeros every 180 degrees
    counter = 0;
    zeroed = 1;
    Serial.println("zero");
  }
}

// Function to display current encoder angle (Tensioning state)
void displayEncoderAngle() {
    int encoderAngle = (counter * 360.0) / cpr;
    Serial.print("Encoder Angle: ");
    Serial.println(encoderAngle);
}

// Function to check if encoder reached 90 degrees
bool encoderReached90Degrees() {
  int encoderAngle = (counter * 360.0) / cpr;
  if (encoderAngle == 90){
    return true;
  } 
  return false;
}

//#include <SoftwareSerial.h>
//#include "RoboClaw.h"
//
//// MOTOR SET UP
//// Set up SoftwareSerial on pins 10 (RX) and 11 (TX)
//SoftwareSerial serial(10, 11);  
//RoboClaw roboclaw(&serial, 10000); // RoboClaw object
//#define address 0x80 // Define the RoboClaw address
//
//// Velocity PID for motor 1, set to zero for position PID
//#define Kp 2
//#define Ki 0.2
//#define Kd 0
//#define qpps 7875
//// Velocity PID for motor 2
//#define Kp 2.35
//#define Ki 0.28
//#define Kd 0
//#define qpps 7875
//
////Position PID
//#define PosKp 2000
//#define PosKi 0
//#define PosKd 50000
//#define KiMax 0
//#define DeadZone 10
//#define Min -10000
//#define Max 10000
//
//// ENCODER SET UP
//volatile int counter = 0; // increase or decrease depending on the rotation of encoder
//float angle = 0;
//float linear_distance = 0;
//const int cpr = 4000;
//const float radius = 5.75; // mm, radius of encoder capstan
//
//
//void setup() {
//  // Initialize serial communication
//  Serial.begin(9600);   // For Serial Monitor
//  roboclaw.begin(38400);  // For RoboClaw communication
//
//
//  roboclaw.SetM1VelocityPID(address,Kp,Ki,Kd,qpps);
//  roboclaw.SetM2VelocityPID(address,Kp,Ki,Kd,qpps);
////  roboclaw.SetM1PositionPID(address,PosKp,PosKi,PosKd,KiMax,DeadZone,Min,Max);
//  
//  // Encoder -- A leads B is clockwise, B leads A in counterclockwise
//   pinMode(2, INPUT_PULLUP); // internal pullup input pin 2, A (brown)
//   pinMode(3, INPUT_PULLUP); // internal pullup input pin 3, B (red)
//   pinMode(18, INPUT_PULLUP);
//
//  //Arduino Mega interrupt pins: 2, 3, 18, 19, 20, 21
//   attachInterrupt(digitalPinToInterrupt(2), A, RISING);
//   attachInterrupt(digitalPinToInterrupt(3), B, RISING);
//   attachInterrupt(digitalPinToInterrupt(18), zero, RISING);
//   
//  // Nothing happens until encoder is initialized at zero
////   while(digitalRead(18)==LOW){
////   } 
////   counter = 0;
//}
//
//
//void loop() {
//  moveMotor();
//}
//
//int start = 0;
//void moveMotor(void) {
//  uint8_t depth1,depth2;
////  roboclaw.SpeedAccelDeccelPositionM1(address, 0, qpps, 0, 6000, 1);
////  roboclaw.SpeedAccelDeccelPositionM1(address, 0, qpps, 0, 3000, 0);
////  if(start == 0){
////    roboclaw.SpeedDistanceM1(address,-2000,600,1);
////    while(true){
////      roboclaw.ReadBuffers(address,depth1, depth2);
////      if (depth1 == 0x80) break;
////    }
////    start++;
////  }
//  
//  
////  roboclaw.SpeedDistanceM1(address,2000,600,1);
////  while(true){
////    roboclaw.ReadBuffers(address,depth1, depth2);
////    readEncoder();
////    readMotorEncoder();
////    Serial.print("\n");
////    if (depth1 == 0x80) break;
////  }
////  roboclaw.SpeedDistanceM1(address,-2000,600,1);
////  while(true){
////    roboclaw.ReadBuffers(address,depth1,depth2);
////    readEncoder();
////    readMotorEncoder();
////    Serial.print("\n");
////    if (depth1==0x80) break;
////  }
//
//
//  roboclaw.SpeedDistanceM2(address,2000,600,1);
//    while(true){
//      roboclaw.ReadBuffers(address,depth1, depth2);
//      readEncoder();
//      readMotorEncoder();
//      Serial.print("\n");
//      if (depth2 == 0x80) break;
//  }
//  roboclaw.SpeedDistanceM2(address,-2000,600,1);
//  while(true){
//    roboclaw.ReadBuffers(address,depth1,depth2);
//    readEncoder();
//    readMotorEncoder();
//    Serial.print("\n");
//    if (depth2==0x80) break;
//  }
//}
//
//void readMotorSpeed(void) {
//  uint8_t status1,status2;
//  bool valid1,valid2;
//  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status1, &valid1);
//  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status2, &valid2);
//  Serial.print("Actual speed1:");
//  Serial.print(speed1);
//  Serial.print("\n");
//}
//
void readMotorEncoder(void) {
  uint8_t status1, status2; // 80 (increase) or 82 (decrease) 
  bool valid1, valid2;
  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  Serial.print("MotorEncoder1:");
  Serial.print(enc1,DEC);
  Serial.print(",");
  Serial.print("MotorEncoder2:");
  Serial.print(enc2,DEC);
  Serial.print(",");
}


void readEncoder(void) {
//   360 degree at bottom, resets to zero there
//    angle = (counter * 360.0) / cpr; // Angle in degrees
//    linear_distance = (angle * PI / 180.0) * radius; // Linear distance in mm
    Serial.print("counter:");
    Serial.print(counter);
    Serial.print(",");
//    Serial.print("angle:");
//    Serial.print(angle); // Two decimal precision
//    Serial.print("\n");
}
