#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "mbed.h"
using namespace mbed;

// address for each sensor corresponding multiplexer channel
#define FRONT_LEFT 0x02
#define FRONT_RIGHT 0x01
#define SIDE_RIGHT 0x04
#define SIDE_LEFT 0x08
// coefficients for the quadratic equation for converting raw value to distance in milimeters
// calibrated specifically for the mazes walls 
#define A -2.813e-8
#define B 0.0105
#define C -7.238
class IR_sensor
{
public:
    // sensor constructor
    IR_sensor(char mux_cmd);
    // return a millimetre distance reading from the sensor 
    float read();

private:
    // array to store the 16bit raw value from sensor in two 8 bit sections
    char _cmd[2];
    // address for the multiplexer 
    const char _mux_addr = 0xEE;
    // command to select the multiplexer channel for the corresponding sensor
    char _mux_cmd;
    // address for the infer red sensor 
    const char _IR_addr = 0x80;
    
};

extern IR_sensor ir;

#endif