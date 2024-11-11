#include "IRSensor.h"
#include "mbed.h"
using namespace mbed;

I2C i2c(P0_31, P0_2);

IR_sensor::IR_sensor(char mux_cmd):_mux_cmd(mux_cmd){

}


int IR_sensor::read(){
    _cmd[0] = 0x5E;
    _cmd[1] = 0x00;
    // Write the mux command to select the correct bus on the multiplexer chip
    i2c.write(_mux_addr, &_mux_cmd, 1);
    // Send command to read distance from sensor
    i2c.write(_IR_addr, _cmd, 1);
    // Wait for sensor to process the command
    wait_us(500000);
    // Read the distance date from the sensor
    i2c.read(_IR_addr, _cmd, 2);
    _decArray[0] = _cmd[0], _cmd[1];
    return((_decArray[0] * 0.25));
}