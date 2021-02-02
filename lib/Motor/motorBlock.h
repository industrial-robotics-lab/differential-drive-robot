#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"


class MotorBlock
{

private:
    Encoder* encoder;

    float distanceTraveled_k1;
    float distanceTraveled_k0;

    byte PWM_PIN;
    float pwm;
    
    float wheelRadius;

    byte IN_DRIVER_PIN_1;
    byte IN_DRIVER_PIN_2;

public:
    MotorBlock();
    ~MotorBlock();

    void createWheel(float wheelRadius);
    
    // SET
    void setEncorerPin(byte encPin);
    void setVelocity(float vel, float maxVel);
    void setDriverPin(byte driverPin1, byte driverPin2, byte driverPinPWM);

    // GET
    float getRadiusWheels();
    float getDistance();
    
    
    // float computeMotorSpeed (float linVel, float angVel, float L);
};
#endif // MOTOR_H