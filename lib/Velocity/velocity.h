#ifndef VELOCITY_H
#define VELOCITY_H
#include <math.h>

class Velocity
{
private:
    
public:
    float ang;
    float lin;
    float maxRobot;
    float maxWheel;

    Velocity();
    float computeLinearSpeed();
    // float 
};

#endif // VELOCITY_H
