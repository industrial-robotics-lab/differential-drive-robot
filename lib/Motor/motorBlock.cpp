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
    pwm = map(abs(vel), 0, maxVel, 150, 255);
    // pwm = map(vel, 0, maxVel, 0, 255); 

    if (vel > 0)
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

float MotorBlock::getTraveledDistance()
{   
    float ovTurn_k0 = encoder->overallTurnEnc_k0;
    float ovTurn_k1 = encoder->getOverallTurn();
    float R = getRadiusWheels();


    // Расчет пройденного расстояния колесом
    distanceTraveled_k1 = distanceTraveled_k0 + 2 * PI * R * (ovTurn_k1 - ovTurn_k0) / 4095.0;
    
    // Обновление значений
    distanceTraveled_k0 = distanceTraveled_k1; // по пройденному расстоянию
    encoder->overallTurnEnc_k0 = ovTurn_k1;    // по абсолютному улглу энкодера

    return distanceTraveled_k1;
}


