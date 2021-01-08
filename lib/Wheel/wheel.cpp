#include "wheel.h"

Wheel::Wheel()
: wheelRadius(0)
{}

void Wheel::create(float wheelRadius){
    this->wheelRadius = wheelRadius;
}

float Wheel::getRadius()
{
    return wheelRadius;
}