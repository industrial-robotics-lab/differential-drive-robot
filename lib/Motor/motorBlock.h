#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"
#include "wheel.h"

class MotorBlock
{

private:
    Encoder* encoder;
    Wheel* wheel;
    
public:
    MotorBlock();
    ~MotorBlock();

    void createWheels(float wheelRadius, float baseLength);
    void setEncorerPin(uint8_t encPin);

};


#endif // MOTOR_H