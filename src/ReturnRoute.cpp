#include "ReturnRoute.h"
#include <iostream>
#include "MazeMapping.h"
#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "mbed.h"
#include "rtos.h"
#include "Encoder.h"
#include "IRSensor.h"

ReturnRoute::ReturnRoute(MazeMapping &mapInfo, MovementControl &movment)
    : _mapInfo(mapInfo),
      _movement(movment)
{}


void ReturnRoute::ReturnToStart()
{
    
    _movement.turnRight(180);
    int heading = 0;
    int x = 20, y = 20; // Assuming the robot starts at (20, 20)
    
}