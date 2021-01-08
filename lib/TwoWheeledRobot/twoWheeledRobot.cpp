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

void TwoWheeledRobot::setPID(float Kp, float Ki, float Kd)
{
   pid->setCoefficient(Kp, Ki, Kd);
}

float TwoWheeledRobot::getRadiusWheels()
{
  return motorBlockL->getRadiusWheels();
}

// Velocity TwoWheeledRobot::getVelocity()
// {       return Velocity {v_r, v_l}
// }

void TwoWheeledRobot::goToGoal(float xGoal, float yGoal, float dt)
{

  //Расчет целевого угла
  thetaGoal = atan2(xGoal-xPos, yGoal-yPos);
  float R = getRadiusWheels();
  float L = baseLength;

  for (uint8_t i = 0; i < 50; i++)
  {
    // расчет ошибки
    float err = pid->computeAngleError(thetaGoal, theta);
  
    vel.ang = pid->computeControl(err, dt);
    vel.lin = vel.computeLinearSpeed(vel.ang, maxVel);
    // vel = getVelocity();

    //Расчет скоростей для каждого двигателся

    float velR = (2*vel.lin + vel.ang*L)/(2*R);
    float velL = (2*vel.lin - vel.ang*L)/(2*R);

    motorBlockL->setVelocity(pinPWM_L, velL);
    motorBlockR->setVelocity(pinPWM_R, velR);

  }
}


