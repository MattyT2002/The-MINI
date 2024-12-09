#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "IRSensor.h"
#include "mbed.h"
#include "Encoder.h"
#include "WallFollowing.h"
using namespace mbed;



int count = 0;

rtos::Thread thread;

void setup()
{
    leftMotor.setup();
    rightMotor.setup();
}
void loop()
{
    wait_us(3000000);
    movementControl.wallFollow(105,105,50);
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
    
    wait_us(3000000);
    movementControl.forward(300);
    movementControl.alignToWall();
    movementControl.turnLeft(90);
    movementControl.alignToWall();
    wait_us(500000);
    movementControl.turnLeft(90);
    movementControl.alignToWall();
    wait_us(500000);
    movementControl.turnRight(90);
    movementControl.alignToWall();
    wait_us(500000);
    movementControl.turnRight(90);
    movementControl.alignToWall();
*/
}