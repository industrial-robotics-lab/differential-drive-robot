#ifndef ENCODER_H
#define ENCODER_H
#include <AS5600.h>
#include "Wire.h"
#include <AS5600.h>
extern "C" { 
#include "utility/twi.h"  
}

// #define TCAADDR 0x70 

class Encoder
{

private:
    AS5600* enc;
    byte encPin;

    int TCAADDR = 112; // адрес (все контакты на землю)

    float initPose;
    float tic_k0; //Значение в (к-1)-ый момент времени
    float tic_k1; //Значение в (к)-ый момент времени
    
    
    int rev;
    
    
public: 
    Encoder();
    ~Encoder();
    long absPosEnc_k1; // абсолютная позиция экнодера в (к)-ый момент времени
    long absPosEnc_k0; // абсолютная позиция экнодера в (к-1)-ый момент времени
    
    // SET
    void setPin(uint8_t encPin);

    // GET 
    float getAbsolutePosition();
    void tcaSelect(uint8_t i);
};

#endif // ENCODER_H