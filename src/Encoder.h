#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"
using namespace mbed;

 
#define DISTANCE_PER_TICK 0.1142
#define RIGHT_ENCODER_A P1_15
#define RIGHT_ENCODER_B P1_11
#define LEFT_ENCODER_A P1_12
#define LEFT_ENCODER_B P1_14

class Encoder{
    public:
    Encoder(PinName pinA, PinName pinB);
    void reset();
    void setup();
    int print();
    float getDistance();
    private:
        void updateEnCount();       

        long int _EnCount;
        
    
        InterruptIn _pinA;
        InterruptIn _pinB;  
        
};

extern Encoder encoder;

#endif