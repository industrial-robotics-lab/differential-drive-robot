<<<<<<< HEAD
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


public:
    TwoWheeledRobot();
    ~TwoWheeledRobot();

    void createWheels(float wheelRadius, float baseLength, float maxVel);

    
    void goToGoal(float x_d, float y_d, float dt);
    void tunePID(float Kp, float Ki, float Kd);

    

    // SET
    void setEncoderPins(byte encPinL, byte encPinR);
    void setDriverPins(byte driverPinL1, byte driverPinL2, byte driverPinR1, byte driverPinR2, byte driverPinPWM1, byte driverPinPWM2);
    // GET
    float getRadiusWheels();
};

=======
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


public:
    TwoWheeledRobot();
    ~TwoWheeledRobot();

    void createWheels(float wheelRadius, float baseLength, float maxVel);

    
    void goToGoal(float x_d, float y_d, float dt);
    void tunePID(float Kp, float Ki, float Kd);

    

    // SET
    void setEncoderPins(byte encPinL, byte encPinR);
    void setDriverPins(byte driverPinL1, byte driverPinL2, byte driverPinR1, byte driverPinR2, byte driverPinPWM1, byte driverPinPWM2);
    // GET
    float getRadiusWheels();
};

>>>>>>> 0e02d83e1e584c678ae587787faadac5daf6b418
#endif // TWO_WHEELED_ROBOT_H