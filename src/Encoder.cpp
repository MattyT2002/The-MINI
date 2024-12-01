#include "Encoder.h"
#include "mbed.h"
using namespace mbed;

Encoder::Encoder(PinName pinA, PinName pinB)
    : _pinA(pinA), _pinB(pinB)
{
}

void Encoder::setup()
{
    _pinA.rise(callback(this, &Encoder::updateEnCount));
}

void Encoder::updateEnCount()
{

    if (_pinA != _pinB)
    {
        _EnCount++;
    }
    else
    {
        _EnCount--;
    }
}

void Encoder::reset()
{
    _EnCount = 0;
}

int Encoder::print()
{
    Serial.println(_EnCount);
}

float Encoder::getDistance()
{
    // returns distance wheel has spin in millimetres.
    return ((_EnCount * DISTANCE_PER_TICK) * 4);
}