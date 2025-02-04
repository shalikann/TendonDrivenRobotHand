#include <SoftwareSerial.h>
#include "RoboClaw.h"

// Set up SoftwareSerial on pins 10 (RX) and 11 (TX)
SoftwareSerial serial(10, 11);  
RoboClaw roboclaw(&serial, 10000); // RoboClaw object

#define address 0x80 // Define the RoboClaw address

#define Kp 1.0
#define Ki 0.1
#define Kd 0.5
#define qpps 7920

volatile int temp, counter = 0; // increase or decrease depending on the rotation of encoder
float angle = 0;
float linear_distance = 0;
const int cpr = 2000;
const float radius = 5.75; // mm, radius of encoder capstan
const int rpm = 100;
const float cpr_motor = 4752.0; // 48*99 
volatile float distance = 0;
int loops = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);   // For Serial Monitor
  roboclaw.begin(38400);  // For RoboClaw communication

  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);

  // Encoder -- A leads B is clockwise, B leads A in counterclockwise
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2, A (brown)
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3, B (red)
  pinMode(18, INPUT_PULLUP);

  //Arduino Mega interrupt pins: 2, 3, 18, 19, 20, 21
  attachInterrupt(digitalPinToInterrupt(2), A, RISING);
  attachInterrupt(digitalPinToInterrupt(3), B, RISING);
  attachInterrupt(digitalPinToInterrupt(18), zero, RISING);

  while(digitalRead(18)==LOW){
  } 
  counter = 0;
//  Serial.print("Counter initialized to zero");
}


void loop() {
  readEncoder();
  // motor move
  if(loops<20){
    roboclaw.ForwardM1(address,30); // counterclockwise
    delay(500);
    roboclaw.BackwardM1(address,30); //clockwise
    delay(500);
  }
  loops++;
//  roboclaw.ForwardM1(address,0);
//  delay(1000);

}

// void readMotorSpeed(void) {
//  uint8_t status2;
//  bool valid2;
//  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
////  Serial.print("Speed1:");
////  if(valid2){
////    Serial.print(speed1,DEC);
////    Serial.print("");
////  } else {
////    Serial.print("failed");
////  }
// }

 void readMotorEncoder(void) {
   //Read all the data from Roboclaw before displaying on Serial Monitor window
   //This prevents the hardware serial interrupt from interfering with
   //reading data using software serial.
   uint8_t status1; // 80 (increase) or 82 (decrease) 
   bool valid1;
   int32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
   Serial.print("MotorEncoder1");
   if(valid1){
     Serial.print(enc1,DEC);
     Serial.print(" ");
     Serial.print(status1,HEX);
     Serial.print(" ");
   }
   else{
     Serial.print("invalid ");
   }
   Serial.println();
 }


void readEncoder(void) {
  // 360 degree at bottom, resets to zero there
  if( counter != temp ){ // only update if position changes
//      Serial.print("\nEncoder:\n");
      angle = (counter * 180.0) / cpr; // Angle in degrees
      linear_distance = (angle * PI / 180.0) * radius; // Linear distance in mm

//      Serial.print("counter:");
//      Serial.print(counter);
//      Serial.print("\n");
//      Serial.print("angle:");
      Serial.print(angle); // Two decimal precision
      Serial.print("\n");
//      Serial.print("linear_distance:");
//      Serial.print(linear_distance);
//      readMotorEncoder();
      temp = counter;
  }
}

void A() {
  // A is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter++;
  }else{
    counter--;
  }
}
   
void B() {
  // B is activated if DigitalPin nr 3 is going from LOW to HIGH
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
