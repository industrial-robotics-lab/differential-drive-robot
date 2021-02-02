#include "twoWheeledRobot.h"
#include "constants.h"

TwoWheeledRobot::TwoWheeledRobot() 
{
  motorBlockL = new MotorBlock();
  motorBlockR = new MotorBlock();
  pid = new PID();
}

TwoWheeledRobot::~TwoWheeledRobot()
{
  delete motorBlockL;
  delete motorBlockR;
  delete pid;
}


void TwoWheeledRobot::createWheels(float wheelRadius, float baseLength, float maxVel)
{
  motorBlockL->createWheel(wheelRadius);
  motorBlockR->createWheel(wheelRadius);
  this->baseLength = baseLength;
  vel.max = 2*PI*wheelRadius*maxVel;
}

void TwoWheeledRobot::setEncoderPins(byte encPinL, byte encPinR)
{
  motorBlockL->setEncorerPin(encPinL);
  motorBlockR->setEncorerPin(encPinR);
}

void TwoWheeledRobot::setDriverPins(byte driverPinL1, byte driverPinL2, byte driverPinR1, byte driverPinR2, byte driverPinPWM1, byte driverPinPWM2)
{
  motorBlockL->setDriverPin(driverPinL1, driverPinL2, driverPinPWM1);
  motorBlockR->setDriverPin(driverPinR1, driverPinR2, driverPinPWM2);
}

void TwoWheeledRobot::tunePID(float Kp, float Ki, float Kd)
{
   pid->setCoefficient(Kp, Ki, Kd);
}


float TwoWheeledRobot::getRadiusWheels()
{
  return motorBlockL->getRadiusWheels();
}


void TwoWheeledRobot::goToGoal(float xGoal, float yGoal, float dt)
{
  //Расчет целевого угла
  pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);
  // Serial.print("pos.thetaGoal: "); Serial.println(pos.thetaGoal); // ----- TEST
  
  float R = getRadiusWheels();
  float L = baseLength;
  
  for (int i = 0; i < 100; i++)
  {
    // расчет ошибки
    float err = pid->computeAngleError(pos.thetaGoal, pos.theta);
    // Serial.print("err: "); Serial.println(err); // ----- TEST
    // Serial.print("theta: "); Serial.println(pos.theta); // ----- TEST
    
    vel.ang = pid->computeControl(err, dt/1000);
    vel.lin = vel.computeLinearSpeed();
    // Serial.print("vel.ang: "); Serial.println(vel.ang);
    // Serial.print("vel.lin: "); Serial.println(vel.lin);


    //Расчет скоростей для каждого двигателся
    float velR = (2*vel.lin + vel.ang*L)/(2*R);
    float velL = (2*vel.lin - vel.ang*L)/(2*R);


    motorBlockL->setVelocity(velL, vel.max);
    motorBlockR->setVelocity(velR, vel.max);
    // Serial.print("velL: "); Serial.println(velL);
    // Serial.print("velR: "); Serial.println(velR);


    float distWheelL = motorBlockL->getDistance();
    float distWheelR = motorBlockR->getDistance();
    float distWheelC = (distWheelR + distWheelL) / 2;
    // Serial.print("distWheelL: "); Serial.println(distWheelL);
    // Serial.print("distWheelR: "); Serial.println(distWheelR);


    pos.computeCurentPose(distWheelL, distWheelR, distWheelC, L);
    // Serial.print("X: "); Serial.println(pos.x);
    // Serial.print("Y: "); Serial.println(pos.y);

    if (DEBUG){
      Serial.print("R: "); Serial.println(distWheelR);
      Serial.print("L: "); Serial.println(distWheelL);
    }
    delay(dt);
  }
}


