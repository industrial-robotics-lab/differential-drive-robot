#ifndef TWO_WHEELED_ROBOT_H
#define TWO_WHEELED_ROBOT_H

#include "motorBlock.h"
#include "pid.h"
#include "velocity.h"
#include "position.h"


class TwoWheeledRobot
{
private:
    MotorBlock* motorBlockL;
    MotorBlock* motorBlockR;
    PID* pid;
    Velocity vel;
    Position pos;

    float baseLength;
    byte PIN_CURRENT_SENSOR = A12;

    bool reachedGoal;

public:
    TwoWheeledRobot();
    ~TwoWheeledRobot();

    void createWheels(float wheelRadius, float baseLength, float maxVel);

    
    
    void tunePID(float Kp, float Ki, float Kd);
    int checkCurrent(byte PIN_CURRENT_SENSOR);

    void manualControl();
    void stopMoving();
    void goToGoal(float x_d, float y_d, float dt);
    void goForward();
    void turnLeft();
    void turnRight();

    // SET
    void setEncoderPins(byte encPinL, byte encPinR);
    void setDriverPins(byte driverPinPWM_R, byte driverPin_R2, byte driverPin_R1, byte driverPin_L1, byte driverPin_L2, byte driverPinPWM_L);
    // GET
    float getRadiusWheels();
};

#endif // TWO_WHEELED_ROBOT_H