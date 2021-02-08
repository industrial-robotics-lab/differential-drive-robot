#include "encoder.h"

Encoder::Encoder()
    :tic_k0(0) , tic_k1(0), rev(0),
    absPosEnc_k1(0), absPosEnc_k0(0)
{
    enc = new AS5600();
    tcaSelect(encPin);
    initPose = enc->getPosition();
    Wire.begin();
}

Encoder::~Encoder()
{
    delete enc;
}


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
float Encoder::getAbsolutePosition()
{   
    tcaSelect(encPin);
    tic_k1 = enc->getPosition() - initPose; // 0-я позиция

    if ((tic_k0 - tic_k1) > 2047)
        rev++;
    if ((tic_k0 - tic_k1) < -2047)
        rev--;
    
    absPosEnc_k1 = rev * 4095.0 + tic_k1;
    tic_k0 = tic_k1;
    return absPosEnc_k1;
}
