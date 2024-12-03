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

    // Write the mux command to select the correct bus on the multiplexer
    if (i2c.write(_mux_addr, &_mux_cmd, 1) != 0)
    {
        printf("Error: Failed to write to multiplexer\n");
        return -1.0f; // Return error value
    }

    // Send the read command to the IR sensor
    if (i2c.write(_IR_addr, _cmd, 1) != 0)
    {
        printf("Error: Failed to write to IR sensor\n");
        return -1.0f; // Return error value
    }

    // Wait for sensor to process the command
    wait_us(500000);

    // Read 2 bytes of distance data from the sensor
    if (i2c.read(_IR_addr, _cmd, 2) != 0)
    {
        printf("Error: Failed to read from IR sensor\n");
        return -1.0f; // Return error value
    }

    // Combine the two bytes of data into a single raw value
    int rawValue = (_cmd[0] << 8) | _cmd[1];

   
    float distance = -2.813e-8 * rawValue*rawValue + 0.0105 * rawValue + -7.238;
    
    // Optional: Implement smoothing (e.g., exponential moving average)
    static float smoothedDistance = distance;  // Initialize with the first reading
    const float alpha = 0.8f;                  // Smoothing factor 
    smoothedDistance = alpha * distance + (1.0f - alpha) * smoothedDistance;
  

    return distance; // Return the smoothed, calibrated distance

}
