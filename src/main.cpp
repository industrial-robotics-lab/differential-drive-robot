#include <Arduino.h>
#include "twoWheeledRobot.h"

//for UNO
//A4 -- SDA
//A5 -- SCL


#define WHEEL_RADIUS 0.09
#define BASE_LENGTH 0.3
#define MAX_VELOCITY 150

#define ENCODER_PIN_L 1
#define ENCODER_PIN_R 2
double dt = 10;

// long rev = 0;         // количество полных оборотов
// long initPose;



void setup() {
  Serial.begin(9600);
  TwoWheeledRobot robot;
  robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
  
  robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
  
  robot.tunePID(0.06, 0.02, 0);
  
  robot.goToGoal(1, 1, dt);
  
}


void loop() {
  // if (Serial.available() > 0) 
  // {
  //   float msg = Serial.read();
  //   robot.goToGoal(1, 1, 0.1);
  // }
  // robot.sendData();
}



// void debug (String name_debug, float val ) {
//   Serial.print(name_debug);
//   Serial.println(val);
// }