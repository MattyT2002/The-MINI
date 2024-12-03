#ifndef MOVEMENTCONTROL_H
#define MOVEMENTCONTROL_H
#include "Arduino.h"
#include "MotorControl.h"
#include "mbed.h"

using namespace mbed;

#define Right_Forward 0
#define Left_Forward 1
#define Right_Backwards 1
#define Left_Backwards 0
#define pi 3.14
#define center_to_wheel 45
#define proportionalResponse 0.07
#define tolerance 10
#define alignSpeed 0.15
class MovementControl
{
public:
    MovementControl(Motor &leftMotor, Motor &rightMotor);

    void stop();
    void forward(float distance);
    void reverse(float distance);
    void turnLeft(int degrees);
    void turnRight(int degrees);
    void alignToWall();
    void wallFollow(float wallDistance, float moveDistance);

private:
    float getacrlength(float degrees);
    Motor &_leftMotor;
    Motor &_rightMotor;
    float _pwr;
    
};

extern MovementControl movementControl;

#endif