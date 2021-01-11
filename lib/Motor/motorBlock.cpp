#include "motorBlock.h"

MotorBlock::MotorBlock()
    : distanceTraveled_k1(0), distanceTraveled_k0(0)
{
    encoder = new Encoder();
    wheel = new Wheel();
}

MotorBlock::~MotorBlock()
{
    delete encoder;
    delete wheel; 
}

void MotorBlock::createWheels(float wheelRadius)
{
    wheel->create(wheelRadius);
}



// === SET ===
void MotorBlock::setEncorerPin(uint8_t encPin)
{
    encoder->setPin(encPin);
}

void MotorBlock::setVelocity(int pinPWM, float vel, float maxVel)
{   
    pwm = map(vel, 0, maxVel, 0, 255);
    analogWrite(pinPWM, pwm);
}

// === GET ===
float MotorBlock::getRadiusWheels()
{
    return wheel->getRadius();
}

float MotorBlock::getDistance()
{   
    float absPos_k1 = encoder->getAbsolutePosition();
    float absPos_k0 = encoder->absPosEnc_k0;
    float R = getRadiusWheels();
    distanceTraveled_k1 = distanceTraveled_k0 - 2 * PI * R * (absPos_k1 - absPos_k0) / 4095.0;
    distanceTraveled_k0 = distanceTraveled_k1;
    encoder->absPosEnc_k0 = absPos_k1;
    return distanceTraveled_k0;
}






// float MotorBlock::computeMotorSpeed (float linVel, float angVel, float L)
// {   
//     float R = getRadiusWheels();
//     return (2*linVel + angVel*L)/(2*R);
// }
