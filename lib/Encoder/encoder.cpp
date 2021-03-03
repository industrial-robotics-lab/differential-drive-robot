#include "encoder.h"

Encoder::Encoder()
    :ang_k0(0) , ang_k1(0), rev(0),
    overallTurnEnc_k1(0), overallTurnEnc_k0(0)
{
    enc = new AS5600();
    tcaSelect(encPin);
    initPose = enc->getPosition();
    Wire.begin();
}

Encoder::~Encoder()
{ delete enc; }


void Encoder::tcaSelect(uint8_t i)
{
    if (i > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}



// === SET ===
void Encoder::setPin(uint8_t encPin)
{
    this->encPin = encPin;
}



// === GET ===
float Encoder::getOverallTurn()
{   
    // Считывание угла поворота
    tcaSelect(encPin);
    ang_k1 = enc->getPosition() - initPose; 
    
    // Проверка на полный оборот
    if ((ang_k0 - ang_k1) > 2047)
        rev++;
    if ((ang_k0 - ang_k1) < -2047)
        rev--;
    
    // Расчет полных оборотов энкодера + текущий угол
    overallTurnEnc_k1 = rev * 4095.0 + ang_k1;

    ang_k0 = ang_k1; // Обновление текущего значения угла
    
    return overallTurnEnc_k1;
}
