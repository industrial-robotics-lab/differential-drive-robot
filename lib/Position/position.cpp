#include "position.h"


Position::Position() 
:   x(0.0), y(0.0), theta(0.0),
    xGoal(0.0), yGoal(0.0), thetaGoal(0.0)
{

}

void Position::computeCurentPose(float D_L, float D_R, float D_C, float L)
{
    x = D_C * cos(theta);
    y = D_C * sin(theta);
    theta = (D_R-D_L)/L;
}