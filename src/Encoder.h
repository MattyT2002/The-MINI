#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"
using namespace mbed;


#define DISTANCE_PER_TICK 0.1142
// pins reffrance for each encoder
#define RIGHT_ENCODER_A P1_15
#define RIGHT_ENCODER_B P1_11
#define LEFT_ENCODER_A P1_12
#define LEFT_ENCODER_B P1_14

class Encoder
{
public:
    Encoder(PinName pinA, PinName pinB);
    // returns the encoder count to 0 
    void reset(); 
    // sets up the encoder interrupt to track the encoder pin rises 
    void setup();
    // prints the current encoder count
    int print();
    // prints the current distance in milimeters that the motor has moved based of encoder counts
    float getDistance();

private:
    // updates the encoder count based off the current PinA and PinB states to calculate the direction of movement of the encount
    void updateEnCount();
    // encoder count
    long int _EnCount;
    // two pins used to track the direction and movement of the motor
    InterruptIn _pinA;
    InterruptIn _pinB;
};

extern Encoder encoder;

#endif