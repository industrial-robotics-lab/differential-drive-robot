#include "velocity.h"

Velocity::Velocity()
: ang(0), lin(0), max(0)
{}

float Velocity::computeLinearSpeed()
{
   return max/square((fabs(ang)+1));
}

