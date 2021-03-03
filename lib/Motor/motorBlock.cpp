#include "motorBlock.h"

MotorBlock::MotorBlock()
    : distanceTraveled_k1(0), distanceTraveled_k0(0), 
    PWM_PIN(0), pwm(0),
    wheelRadius(0)
{
    encoder = new Encoder();
    pinMode (IN_DRIVER_PIN_1,  OUTPUT);
    pinMode (IN_DRIVER_PIN_2,  OUTPUT);
}

MotorBlock::~MotorBlock()
{
    delete encoder;
}

void MotorBlock::createWheel(float wheelRadius)
{
    this->wheelRadius = wheelRadius;
}

void MotorBlock::stopMoving()
{   
    for (int i = pwm; i != 0 ;)
    {
        i=i-10;
        i=(i<0)?0:i;
        analogWrite(PWM_PIN, i);
        // Serial.println(i);
    }
}

// === SET ===
void MotorBlock::setEncorerPin(byte encPin)
{
    encoder->setPin(encPin);
}

void MotorBlock::setDriverPin(byte driverPin1, byte driverPin2, byte driverPinPWM)
{
    IN_DRIVER_PIN_1 = driverPin1;
    IN_DRIVER_PIN_2 = driverPin2;
    PWM_PIN = driverPinPWM;
}

void MotorBlock::setVelocity(float vel, float maxVel)
{   
    pwm = map(vel, 0, maxVel, 150, 255);

    if (vel >= 0)
    {
        digitalWrite(IN_DRIVER_PIN_1, LOW);
        digitalWrite(IN_DRIVER_PIN_2, HIGH);
    } 
    else 
    {
        digitalWrite(IN_DRIVER_PIN_1, HIGH);
        digitalWrite(IN_DRIVER_PIN_2, LOW);
    }
    analogWrite(PWM_PIN, pwm);
}



// === GET ===
float MotorBlock::getRadiusWheels()
{
    return wheelRadius;
}

float MotorBlock::getDistance()
{   
    float absPos_k1 = encoder->getAbsolutePosition();
    float absPos_k0 = encoder->absPosEnc_k0;
    float R = getRadiusWheels();

    distanceTraveled_k1 = distanceTraveled_k0 + 2 * PI * R * (absPos_k1 - absPos_k0) / 4095.0;
    
    // Обновление значений
    distanceTraveled_k0 = distanceTraveled_k1;
    encoder->absPosEnc_k0 = absPos_k1;
    return distanceTraveled_k0;
}






// float MotorBlock::computeMotorSpeed (float linVel, float angVel, float L)
// {   
//     float R = getRadiusWheels();
//     return (2*linVel + angVel*L)/(2*R);
// }
