#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"
using namespace mbed;

#define RIGHT_ENCODER_A
#define RIGHT_ENCODER_B


class Encoder{
    public:
    Encoder(PinName pinA, PinName pinB);
    void reset();
    private:
        void countPulseA();
        void countPulseB();
        long int _EncCountA;
        long int _EncCountB;
        
        long int _ShaftRevA;
        long int _ShaftRevB; 
        
};

extern Encoder encoder;

#endif