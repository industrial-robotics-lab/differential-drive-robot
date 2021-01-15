#ifndef WHEEL_H
#define WHEEL_H


class Wheel
{

private:
    float wheelRadius;
    
public:
    
    Wheel();
    void create(float wheelRadius);
    float getRadius();
};


#endif // WHEEL_H