#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "IRSensor.h"
#include "mbed.h"
#include "Encoder.h"
using namespace mbed;


IR_sensor IRSideRight(SIDE_RIGHT);
IR_sensor IRSideLeft(SIDE_LEFT);
IR_sensor FrontRight(FRONT_RIGHT);
IR_sensor FrontLeft(FRONT_LEFT);

int count = 0;

rtos::Thread thread;

void setup()
{
    leftMotor.setup();
    rightMotor.setup();
}
void loop()
{
    /*
    while (true)
    {
        Serial.print("Right | ");
        Serial.println(FrontRight.read());
        Serial.print("left | ");
        Serial.println(FrontLeft.read());
        
        

    }
    
   

    
    wait_us(3000000);
    movementControl.forward(100);
    wait_us(3000000);
    movementControl.reverse(100);
    wait_us(2000000);
    movementControl.turnLeft(90);
    */
    wait_us(3000000);
    movementControl.turnLeft(90);
    wait_us(1000000);
    movementControl.alignToWall();
}