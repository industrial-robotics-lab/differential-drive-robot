#ifndef TWO_WHEELED_ROBOT_H
#define TWO_WHEELED_ROBOT_H


#include "motorBlock.h"
#include "pid.h"
#include "velocity.h"


class TwoWheeledRobot
{
private:
    MotorBlock* motorBlockL;
    MotorBlock* motorBlockR;
    PID* pid;
    Velocity vel;

    float baseLength;
    float thetaGoal;
    float xPos;
    float yPos;
    float theta;
    float maxVel;

    int pinPWM_L;
    int pinPWM_R;

public:
    TwoWheeledRobot();
    ~TwoWheeledRobot();

    void createWheels(float wheelRadius, float baseLength, float maxVel);
    float getRadiusWheels();
    void setEncoderPins(uint8_t encPinL, uint8_t encPinR);
    void goToGoal(float x_d, float y_d, float dt);
    void tunePID(float Kp, float Ki, float Kd);

    float computeLinearSpeed(float velAng, float maxVel);
};

#endif // TWO_WHEELED_ROBOT_H