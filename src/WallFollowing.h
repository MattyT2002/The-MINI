#ifndef WALLFOLLOWING_H
#define WALLFOLLOWING_H

#include "IRSensor.h"
#include "MovementControl.h"

#define GRID_SIZE_X 145
#define GRID_SIZE_Y 200

 
class WallFollowing
{
public:
    WallFollowing(MovementControl &movement, IR_sensor &leftSideIR, IR_sensor &rightSideIR, IR_sensor &frontLeftIR, IR_sensor &frontRightIR);
    void followLeftWall(float setDistance, float moveDistance, int buffer);
    
private:
    MovementControl &_movement;
    IR_sensor &_leftSideIR;   // Sensor for detecting left wall
    IR_sensor &_rightSideIR;  // Sensor for detecting right wall
    IR_sensor &_frontLeftIR;  // Sensor for detecting front-left obstacles
    IR_sensor &_frontRightIR; // Sensor for detecting front-right obstacles

    bool canTurnLeft(float threshold);
    bool canTurnRight(float threshold);
    bool canMoveForward(float threshold);
    bool isTooCloseToWall(float setDistance);
    bool shouldTurnLeft(int currentHeading, float leftIR, float rightIR, float threshold);
    void alignToWall();
    void moveAwayFromWall(float moveDistance);
    void moveForward(float distance, int heading, int &currentDistance);
    void initialiseOccupancyGrid();
    void printOccupancyGrid();
};

#endif
