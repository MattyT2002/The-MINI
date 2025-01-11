#include "Encoder.h"
#include "mbed.h"
using namespace mbed;

Encoder::Encoder(PinName pinA, PinName pinB)

    : _pinA(pinA), _pinB(pinB)
{
}

// setup a interrupt for when a rise is seen on Pin A leading to the updateEnCountFunction being called.
void Encoder::setup()
{
    _pinA.rise(callback(this, &Encoder::updateEnCount));
}

void Encoder::updateEnCount()
{
    // if pinA is not the same as pinB this means that pinA is leading to that means the motor and wheel has moved forward
    if (_pinA != _pinB)
    {
        // iterate the encoder count up one to represent the forward movement
        _EnCount++;
    }
    // if pinA and pinB are the same this means that PinB is leading due to A chatching up after it has risen this means the motor and wheel has moved backwards
    else
    {
        // iterate the encoder count down one to represent the backwards movement
        _EnCount--;
    }
}

// resent the encoder count to 0
void Encoder::reset()
{
    _EnCount = 0;
}

// print the current encoder count
int Encoder::print()
{
    Serial.println(_EnCount);
}

// prints the distance in maltimeter the motors has moved using the current encoder count
float Encoder::getDistance()
{
    // returns distance wheel has moved in millimetres. using the current encoder count the distance it should move per tick and then the *4 is a number from calibration
    return ((_EnCount * DISTANCE_PER_TICK) * 4);
}