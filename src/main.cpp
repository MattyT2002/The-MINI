#include "Arduino.h"

#include "MovementControl.h"

using namespace mbed;




rtos::Thread thread;

void setup()
{
    leftMotor.setup();
    rightMotor.setup();
}
void loop()
{
    wait_us(3000000);
    // tell the robot to travers maze and map as it does so
    movementControl.mapThroughMaze(50,55, 20, 20);
    Serial.println("wall follow done");
     wait_us(20000000);
    //movementControl.returnUsingMap();
 
  
}