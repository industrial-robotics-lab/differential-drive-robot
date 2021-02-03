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
  if (DEBUG){
    Serial.print("vel.max: "); Serial.println(vel.max);
  }
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


// ======= GO ======== //
void TwoWheeledRobot::goToGoal(float xGoal, float yGoal, float dt)
{
  //Расчет целевого угла
  pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);
  if (DEBUG){
    Serial.print("pos.thetaGoal: "); Serial.println(pos.thetaGoal); // ----- TEST
  }

  float R = getRadiusWheels();
  float L = baseLength;
  
  for (int i = 0; i <= 60; i++) // ======== FOR
  {
    // расчет ошибки
    float err = pid->computeAngleError(pos.thetaGoal, pos.theta);
    if (DEBUG){
      Serial.print("err: "); Serial.print(err);
    }

    vel.ang = pid->computeControl(err, dt/1000);
    vel.lin = vel.computeLinearSpeed();
    if (DEBUG){
      Serial.print("  vel.ang: "); Serial.print(vel.ang);
      Serial.print("  vel.lin: "); Serial.println(vel.lin);
    }

    //Расчет скоростей для каждого двигателя
    float velR = (2*vel.lin + vel.ang*L)/(2*R);
    float velL = (2*vel.lin - vel.ang*L)/(2*R);
    if (DEBUG){
      Serial.print("velL: "); Serial.print(velL);
      Serial.print("  velR: "); Serial.println(velR);
    }

    motorBlockL->setVelocity(velL, vel.max);
    motorBlockR->setVelocity(velR, vel.max);


    float distWheelL = motorBlockL->getDistance();
    float distWheelR = motorBlockR->getDistance();
    float distWheelC = (distWheelR + distWheelL) / 2;
    if (DEBUG){
      Serial.print("distWheelL: "); Serial.print(distWheelL, 3);
      Serial.print("  distWheelR: "); Serial.print(distWheelR, 3);
      Serial.print("  distWheelC: "); Serial.println(distWheelC, 3);
    }

    pos.computeCurentPose(distWheelL, distWheelR, distWheelC, L);
    if (1){
      Serial.print("X: "); Serial.print(pos.x, 3);
      Serial.print("  Y: "); Serial.print(pos.y, 3);
      Serial.print("  Th: "); Serial.println(pos.theta, 3);
    }

    if((abs(pos.x-xGoal) < 0.03) && (abs(pos.y-yGoal) < 0.03))
    {
      Serial.print("err_X: "); Serial.print(pos.x-xGoal, 3);
      Serial.print("  err_Y: "); Serial.println(pos.y-yGoal, 3);
      motorBlockL->stopMoving();
      motorBlockR->stopMoving();
      break;
    }

    delay(dt);
  }
}


