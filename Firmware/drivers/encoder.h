#ifndef ENCODER_H
#define ENCODEr_H


#include "../common/common.h"

/*
Hall effect encoder 12 CPR

*/




#define PI  3.14159

typedef struct Encoder {
    GPIO chA;
    GPIO chB;
    uint32_t cpr; // counts per revolution
    uint32_t timPerif;
    uint32_t count;
    float *gRatio;
}Encoder;



static Encoder initEncoder(uint32_t timPerif, uint32_t pinA, uint32_t portA, uint32_t pinB, uint32_t portB ){
    Encoder e;
    e.chA.pin = pinA;
    e.chA.port = portA;
    e.chB.pin = pinB;
    e.chB.pin = portB;
    return e;
}


static uint16_t readEncoder(Encoder *e){

}
#endif // ENCODER_H