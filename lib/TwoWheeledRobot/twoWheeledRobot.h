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
    byte PIN_CURRENT_SENSOR;

    bool reachedGoal;
    int newMinRahge; // Для функции map в setVelocity
    byte inByte;

public:
    TwoWheeledRobot();
    ~TwoWheeledRobot();

    void createWheels(float wheelRadius, float baseLength, float maxVel);
    
    // SET
    void setEncoderPins(byte encPinL, byte encPinR);
    void setDriverPins(byte driverPinPWM_R, byte driverPin_R2, byte driverPin_R1, byte driverPin_L1, byte driverPin_L2, byte driverPinPWM_L);
    // GET
    float getRadiusWheels();
    byte getSerialData();
    
    void tunePID(float Kp, float Ki, float Kd);
    
// ========= behavior ===========
    void serialControl();
    void goToGoal(float x_d, float y_d, float dt);
    void manualControl();


    void goForward(int velL, int velR);
    void turnLeft(int velL, int velR);
    void turnRight(int velL, int velR);
    void stopMoving();
    int checkCurrent(byte PIN_CURRENT_SENSOR);
};

#endif // TWO_WHEELED_ROBOT_H