#include "position.h"


Position::Position() 
:   x(0.0), y(0.0), theta(0.0),
    xGoal(0.0), yGoal(0.0), thetaGoal(0.0)
{}

void Position::computeCurentPose(float D_L, float D_R, float D_C, float L)
{   
    float cos_th = 0.0;
    float sin_th = 0.0;
    if (theta == 3.1415/2)
        cos_th = 0.0;
    else
        cos_th = cos(theta);
    
    if (theta == 0.0)
        sin_th = 0.0;
    else
        sin_th = sin(theta);


    x = D_C * cos_th;   
    y = D_C * sin_th;
    theta = (D_R-D_L)/L;
}