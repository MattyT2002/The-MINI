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
    Serial.print(_mapInfo.occupancyGrid[20][20]);
    _movement.turnRight(180);
    int heading = 0;

    

}
