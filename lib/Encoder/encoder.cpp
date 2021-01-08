#include "encoder.h"

Encoder::Encoder()
{
    enc = new AS5600();
}

Encoder::~Encoder()
{
    delete enc;
}

void Encoder::setPin(uint8_t encPin)
{
    this->encPin = encPin;
}