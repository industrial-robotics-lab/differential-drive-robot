#include <Arduino.h>
#include "twoWheeledRobot.h"

//for UNO
//A4 -- SDA
//A5 -- SCL


#define WHEEL_RADIUS 0.09
#define BASE_LENGTH 0.3
#define MAX_VELOCITY 0.1

#define ENCODER_PIN_L 1
#define ENCODER_PIN_R 2
double dt = 0.1;

long rev = 0;         // количество полных оборотов
long initPose;


double posRk0 = 0;       // позиция энкодера в момент k
double posRk1 = 0;     // позиция энкодера в момент k-1

TwoWheeledRobot robot;

void setup() {
  Serial.begin(9600);
  robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
  robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
  robot.setPID(0.06, 0.02, 0);

  // считывание позиции с Raspberry
  // Serial.read();
  // Передача требуемой позиции

}


void loop() {
  if (Serial.available() > 0) 
  {
    float msg = Serial.read();
    robot.goToGoal(1, 1, 0.1);
  }
  // robot.sendData();
}

void debug (String name_debug, double val ) {
  Serial.print(name_debug);
  Serial.println(val);
}