#include <Arduino.h>
#include "twoWheeledRobot.h"
#include "constants.h"

byte dt = 50;

void setup() {
  Serial.begin(9600);
  
  TwoWheeledRobot robot;
  robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
  
  robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
  robot.setDriverPins(DRIVER_IN_A2, DRIVER_IN_A1 , DRIVER_IN_B1, DRIVER_IN_B2, DRIVER_PWM_PIN_A, DRIVER_PWM_PIN_B);
  robot.tunePID(1, 0, 0.0);

  float xGoal = 1;
  float yGoal = 1;
  robot.goToGoal(xGoal, yGoal, dt);

  // robot.goForward();
  // robot.turnLeft();
  // robot.turnRight();
}

void loop() {

}