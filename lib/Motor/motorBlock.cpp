#include "motorBlock.h"

MotorBlock::MotorBlock()
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

void MotorBlock::setEncorerPin(uint8_t encPin)
{
    encoder->setPin(encPin);
}

float MotorBlock::getRadiusWheels()
{
    return wheel->getRadius();
}

void MotorBlock::setVelocity(int pinPWM, float vel)
{   
    pwm = map(vel, 0, 1024, 0, 255);
    analogWrite(pinPWM, pwm);
}