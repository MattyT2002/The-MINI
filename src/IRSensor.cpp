#include "IRSensor.h"
#include "mbed.h"
using namespace mbed;

I2C i2c(P0_31, P0_2);

IR_sensor::IR_sensor(char mux_cmd) : _mux_cmd(mux_cmd)
{
}



float IR_sensor::read()
{
    _cmd[0] = 0x5E;  // Command to initiate reading
    _cmd[1] = 0x00;
    float distanceAdded = 0;

        if (i2c.write(_mux_addr, &_mux_cmd, 1) != 0) {
            printf("Error: Failed to write to multiplexer\n");
            return -1.0f;
        }
        printf("Multiplexer channel selected\n");

        if (i2c.write(_IR_addr, _cmd, 1) != 0) {
            printf("Error: Failed to write to IR sensor\n");
            return -1.0f;
        }
        printf("IR sensor command sent\n");

        wait_us(200000);

        if (i2c.read(_IR_addr, _cmd, 2) != 0) {
            printf("Error: Failed to read from IR sensor\n");
            return -1.0f;
        }
        printf("Raw data: 0x%02X 0x%02X\n", _cmd[0], _cmd[1]);

        int rawValue = (_cmd[0] << 8) | _cmd[1];
        printf("Raw value: %d\n", rawValue);

        float distance = -2.813e-8 * rawValue * rawValue + 0.0105 * rawValue + -7.238;
        
    

    return distance ; // Average distance
}
