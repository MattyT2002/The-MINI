#ifndef WALLFOLLOWING_H
#define WALLFOLLOWING_H

#include "IRSensor.h"
#include "MovementControl.h"

class WallFollowing
{
public:
    WallFollowing(MovementControl &movement, IR_sensor &leftSideIR, IR_sensor &rightSideIR, IR_sensor &frontLeftIR, IR_sensor &frontRightIR);
    void followLeftWall(float setDistance, float moveDistance);

private:
    MovementControl &_movement;
    IR_sensor &_leftSideIR;   // Sensor for detecting left wall
    IR_sensor &_rightSideIR;  // Sensor for detecting right wall
    IR_sensor &_frontLeftIR;  // Sensor for detecting front-left obstacles
    IR_sensor &_frontRightIR; // Sensor for detecting front-right obstacles

    bool canTurnLeft(float threshold);
    bool canMoveForward(float threshold);
    bool isTooCloseToWall(float setDistance);
    void alignToWall();
    void moveAwayFromWall(float moveDistance);
};

#endif
