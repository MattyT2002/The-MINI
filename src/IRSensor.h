#ifndef IRSENSOR_H
#define IRSENSOR_H

#include "mbed.h"
using namespace mbed;

#define FRONT_LEFT 0x02
#define FRONT_RIGHT 0x01

class IR_sensor{
    public:
    IR_sensor(char mux_cmd);
    int read();

    private:
    char _cmd[2];
    const char _mux_addr = 0xEE;
    char _mux_cmd;
    const char _IR_addr = 0x80;
    char _decArray[1];
    

};

extern IR_sensor ir;

#endif