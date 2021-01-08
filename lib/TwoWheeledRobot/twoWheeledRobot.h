#ifndef TWO_WHEELED_ROBOT_H
#define TWO_WHEELED_ROBOT_H


#include "motorBlock.h"
#include "pid.h"
#include "velocity.h"


class TwoWheeledRobot
{
private:
    MotorBlock* motorL;
    MotorBlock* motorR;
    PID* pid;
    Velocity vel;
    

    float thetaGoal;
    float xPos;
    float yPos;
    float theta;
    float maxVel;


public:
    TwoWheeledRobot();
    ~TwoWheeledRobot();

    void createWheels(float wheelRadius, float baseLength, float maxVel);
    void setEncoderPins(uint8_t encPinL, uint8_t encPinR);
    void goToGoal(float x_d, float y_d, float dt);
    void setPID(float Kp, float Ki, float Kd);
    // Velocity getVelocity();
};
#endif // TWO_WHEELED_ROBOT_H