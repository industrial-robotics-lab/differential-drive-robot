#ifndef ENCODER_H
#define ENCODER_H
#include <AS5600.h>
#include "Wire.h"
#include <AS5600.h>
extern "C" { 
#include "utility/twi.h"  
}

class Encoder
{

private:
    AS5600* enc;
    byte encPin;

    int TCAADDR = 112; // адрес (все контакты на землю)

    float initPose;
    float ang_k0; //Значение в (к-1)-ый момент времени
    float ang_k1; //Значение в (к)-ый момент времени

    int rev;
    
    
public: 
    Encoder();
    ~Encoder();
    long overallTurnEnc_k1; // абсолютная позиция экнодера в (к)-ый момент времени
    long overallTurnEnc_k0; // абсолютная позиция экнодера в (к-1)-ый момент времени
    
    // SET
    void setPin(uint8_t encPin);

    // GET 
    float getOverallTurn();
    void tcaSelect(uint8_t i);
};

#endif // ENCODER_H