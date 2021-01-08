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

void MotorBlock::createWheels(float wheelRadius, float baseLength)
{
    wheel->create(wheelRadius, baseLength);
}

void MotorBlock::setEncorerPin(uint8_t encPin)
{
    encoder->setPin(encPin);
}