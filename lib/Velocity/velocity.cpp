#include "velocity.h"

Velocity::Velocity()
: ang(0), lin(0)
{}

float Velocity::computeLinearSpeed(float velAng, float maxVel)
{
   return maxVel/square((fabs(velAng)+1));
}

