#ifndef MOVEMENTCONTROL_H
#define MOVEMENTCONTROL_H
#include "Arduino.h"
#include "MotorControl.h"
#include "mbed.h"

using namespace mbed;

// the number to send to the direction pin to move the corresponding 
// motor forward or backwards
#define Right_Forward 0
#define Left_Forward 1
#define Right_Backwards 1
#define Left_Backwards 0
#define pi 3.14
// distances in milimetrs from the center of the robot 
// to the middle of the wheel used for calculating archlength
#define center_to_wheel 42
// proportional responds for chaning motor pwm to keep motors spinning at same speed
#define proportionalResponse 0.1
// the error difference the sensor are allowed to have when aligning to a wall infront
#define tolerance 0.5
// the PWM for the motors when aligning to a wall
#define alignSpeed 0.15
class MovementControl
{
public:
    // movement control constructor
    MovementControl(Motor &leftMotor, Motor &rightMotor);
    
    // stops the robots movements
    void stop();
    // moves the robot forwards a set millimetre distance
    void forward(float distance);
    // moves the robot backwards a set millimetre distance 
    void reverse(float distance);
    // turn the robot on the spot left a set angle
    void turnLeft(int degrees);
    // turn the robot on the spot right a set angle
    void turnRight(int degrees);
    // aling the robot with the wall infront to make the robot inline with the mazes walls
    void alignToWall();
    // behaviour to map go throught the maze mapping while it does os 
    void mapThroughMaze( float moveDistance, int buffer, int startX, int startY);
    // behaviour for returning through the maze to the start using a occupancy grid (map)
    void returnUsingMap();
private:
    float getacrlength(float degrees);
    Motor &_leftMotor;
    Motor &_rightMotor;
    float _pwr;
    
};

extern MovementControl movementControl;

#endif