#include "pid.h"

PID::PID() 
: Kp(0.0), Ki(0.0), Kd(0.0),
  errOld(0.0), errSum(0.0), errDot(0.0)
{}

PID::~PID()
{}


float PID::computeControl(float err, float dt)
{
    errDot = err - errOld;
    errSum = err + errSum;

    float u = Kp*err + Ki*errSum*dt + Kd*errOld/dt;
    errOld = err;
    return u;
}

float PID::computeAngleError(float thetaGoal, float theta)
{
    return thetaGoal - theta;
    // return atan2(sin(e), cos(e));
}

void PID::setCoefficient(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}