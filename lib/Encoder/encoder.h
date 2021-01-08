#ifndef ENCODER_H
#define ENCODER_H
#include <AS5600.h>
#include "Wire.h"
#include <AS5600.h>
extern "C" { 
#include "utility/twi.h"  
}

#define TCAADDR 0x70 // адрес (все контакты на землю)

class Encoder
{

private:
    AS5600* enc;
    uint8_t encPin;

public:
    Encoder();
    ~Encoder();

    void setPin(uint8_t encPin);


};

#endif // ENCODER_H