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
public:
    MotorBlock();
    ~MotorBlock();

    void createWheels(float wheelRadius);
    
    void setEncorerPin(uint8_t encPin);
    void setVelocity(int pinPWM, float vel);

    float getRadiusWheels();
};


#endif // MOTOR_H