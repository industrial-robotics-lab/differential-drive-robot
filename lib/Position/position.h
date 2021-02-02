#ifndef POSITION_H
#define POSITION_H
#include "math.h"


class Position{
private:
    
public:
    float x;
    float y;
    float theta;

    float xGoal;
    float yGoal;
    float thetaGoal;

    Position();
    void computeCurentPose(float D_L, float D_R, float D_C, float L);
};


#endif // POSITION_H