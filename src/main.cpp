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
    movementControl.MapThroughMaze(110,110,55);
    Serial.println("wall follow done");
     wait_us(20000000);
    //movementControl.returnUsingMap();
 
  
}