#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"
#include "wheel.h"

class MotorBlock
{

private:
    Encoder* encoder;
    Wheel* wheel;
    int pinPWM;
    float pwm;
    float distanceTraveled_k1;
    float distanceTraveled_k0;
public:
    MotorBlock();
    ~MotorBlock();

    void createWheels(float wheelRadius);
    
    // SET
    void setEncorerPin(uint8_t encPin);
    void setVelocity(int pinPWM, float vel, float maxVel);
    
    // GET
    float getRadiusWheels();
    float getDistance();
    
    
    // float computeMotorSpeed (float linVel, float angVel, float L);
};


#endif // MOTOR_H