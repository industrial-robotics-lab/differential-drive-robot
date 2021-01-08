#include "twoWheeledRobot.h"


TwoWheeledRobot::TwoWheeledRobot() 
: xPos(0.0), yPos(0.0)
{
  motorL = new MotorBlock();
  motorR = new MotorBlock();
  pid = new PID();
}

TwoWheeledRobot::~TwoWheeledRobot()
{
  delete motorL;
  delete motorR;
  delete pid;
}


void TwoWheeledRobot::createWheels(float wheelRadius, float baseLength, float maxVel)
{
  motorL->createWheels(wheelRadius, baseLength);
  motorR->createWheels(wheelRadius, baseLength);
  this->maxVel = maxVel;
}

void TwoWheeledRobot::setEncoderPins(uint8_t encPinL, uint8_t encPinR)
{
  motorL->setEncorerPin(encPinL);
  motorR->setEncorerPin(encPinR);
}

void TwoWheeledRobot::setPID(float Kp, float Ki, float Kd)
{
   pid->setCoefficient(Kp, Ki, Kd);
}

// Velocity TwoWheeledRobot::getVelocity()
// {       return Velocity {v_r, v_l}
// }

void TwoWheeledRobot::goToGoal(float xGoal, float yGoal, float dt)
{

  //Расчет целевого угла
  thetaGoal = atan2(xGoal-xPos, yGoal-yPos);

  for (uint8_t i = 0; i < 50; i++)
  {
    // расчет ошибки
    float err = pid->computeAngleError(thetaGoal, theta);
  
  
    vel.ang = pid->computeControl(err, dt);
    vel.lin = vel.computeLinearSpeed(vel.ang, maxVel);
    // vel = getVelocity();
    float L = baseLength;
    float velR = (2*vel.lin + vel.ang*L)/(2*R);
    float velL = (2*vel.lin - vel.ang*L)/(2*R);
        
  }
  
}


