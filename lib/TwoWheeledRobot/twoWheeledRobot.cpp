#include "twoWheeledRobot.h"


TwoWheeledRobot::TwoWheeledRobot() 
: xPos(0.0), yPos(0.0)
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
  motorBlockL->createWheels(wheelRadius);
  motorBlockR->createWheels(wheelRadius);
  this->baseLength = baseLength;
  this->maxVel = maxVel;
}

void TwoWheeledRobot::setEncoderPins(uint8_t encPinL, uint8_t encPinR)
{
  motorBlockL->setEncorerPin(encPinL);
  motorBlockR->setEncorerPin(encPinR);
}

void TwoWheeledRobot::tunePID(float Kp, float Ki, float Kd)
{
   pid->setCoefficient(Kp, Ki, Kd);
}

float TwoWheeledRobot::getRadiusWheels()
{
  return motorBlockL->getRadiusWheels();
}

float TwoWheeledRobot::computeLinearSpeed(float velAng, float maxVel)
{
   return maxVel/square((fabs(velAng)+1));
}



void TwoWheeledRobot::goToGoal(float xGoal, float yGoal, float dt)
{
  //Расчет целевого угла
  thetaGoal = atan2(xGoal-xPos, yGoal-yPos);
  float R = getRadiusWheels();
  float L = baseLength;
  
  for (uint8_t i = 0; i < 100; i++)
  {
    // расчет ошибки
    float err = pid->computeAngleError(thetaGoal, theta);
    //  Serial.print("vel.lin "); Serial.println(vel.lin); // ----- TEST
    vel.ang = pid->computeControl(err, dt);
    vel.lin = computeLinearSpeed(vel.ang, maxVel);
    

    //Расчет скоростей для каждого двигателся

    float velR = (2*vel.lin + vel.ang*L)/(2*R);
    float velL = (2*vel.lin - vel.ang*L)/(2*R);
    // float velL = motorBlockL->computeMotorSpeed(vel.lin, vel.ang, baseLength);
    // float velR = motorBlockR->computeMotorSpeed(vel.lin, vel.ang, baseLength);
    motorBlockL->setVelocity(pinPWM_L, velL, maxVel);
    motorBlockR->setVelocity(pinPWM_R, velR, maxVel);



    float distWheelL = motorBlockL->getDistance();
    float distWheelR = motorBlockR->getDistance();
    float distWheelC = (distWheelR + distWheelL) / 2;
    
    Serial.print("L: "); Serial.println(distWheelR);
    // Serial.print("R: "); Serial.println(distWheelR);
    delay(300);
  }
}


