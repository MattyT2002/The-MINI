#include "IRSensor.h"
#include "mbed.h"
using namespace mbed;

// initalize the I2C pins for the multiplexer
I2C i2c(P0_31, P0_2);

// Constructor for the Infer red sensor
// initialise the muliplexer channel in which this sensor is connected to 
IR_sensor::IR_sensor(char mux_cmd) : _mux_cmd(mux_cmd)
{
}


// function returns distance read from the sensor in millimeters
float IR_sensor::read()
{
    _cmd[0] = 0x5E; // Command to the multiplex to read the first most significant 8 bits of the 16 bit total reading
    _cmd[1] = 0x00; // Command to the multiplex to read the last lest significant 8 bits of the 16 bits total reading 
   
        // select the multiplex address 
        if (i2c.write(_mux_addr, &_mux_cmd, 1) != 0) {
            printf("Error: Failed to write to multiplexer\n");
            return -1.0f; // return error if failed to communicate to the multiplex
        }
        printf("Multiplexer channel selected\n");
        
        // send command to the IR sensor to take a reading
        if (i2c.write(_IR_addr, _cmd, 1) != 0) {
            printf("Error: Failed to write to IR sensor\n");
            return -1.0f; // return error if command fails
        }
        printf("IR sensor command sent\n");
        
        // wait for sensor to take a reading 
        wait_us(20000);

        // read the results from the sensor reading
        if (i2c.read(_IR_addr, _cmd, 2) != 0) {
            printf("Error: Failed to read from IR sensor\n");
            return -1.0f; // return error if cannot read sensor
        }
        

        // create a 16 bit value by shifting _cmd[0] left 8 to the high 8 bits then combine with the _cmd[1] filling out the lower 8 bits
        int rawValue = (_cmd[0] << 8) | _cmd[1];
        printf("Raw value: %d\n", rawValue);

        // apply a quadratic equation calibrated (using python script) for taking sensor reading in the maze to convert 
        //raw distance value into a millimeter distance reading 
        float distance = A * rawValue * rawValue + B * rawValue + C;
        
    

    return distance ; // Average distance
}
