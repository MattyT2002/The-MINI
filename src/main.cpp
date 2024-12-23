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
    movementControl.wallFollow(105,105,55);
    Serial.println("wall follow done");
     wait_us(30000000);
  
}