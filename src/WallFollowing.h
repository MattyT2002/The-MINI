#ifndef WALLFOLLOWING_H
#define WALLFOLLOWING_H

#include "IRSensor.h"
#include "MovementControl.h"

#define GRID_SIZE_X 145
#define GRID_SIZE_Y 200
#define GRID_CELL_SIZE 10
#define ROBOT_SIZE 150 // robot size in mm in length and width
#define ROBOT_CELLS 15 // robot takes up a 3x3 grid size on the occupancy grd

#define GRID_FORWARD 0 // heading representing the direction the robot is facing and going in relation to the occupancy grid and the desired direction
#define GRID_RIGHT 90
#define GRID_BACKWARDS 180
#define GRID_LEFT 270
class WallFollowing
{
public:
    WallFollowing(MovementControl &movement, IR_sensor &leftSideIR, IR_sensor &rightSideIR, IR_sensor &frontLeftIR, IR_sensor &frontRightIR);
    void followLeftWall(float setDistance, float moveDistance, int buffer);
    void returnToStart();
    int occupancyGrid[GRID_SIZE_X][GRID_SIZE_Y];
private:
    MovementControl &_movement;
    IR_sensor &_leftSideIR;   // Sensor for detecting left wall
    IR_sensor &_rightSideIR;  // Sensor for detecting right wall
    IR_sensor &_frontLeftIR;  // Sensor for detecting front-left obstacles
    IR_sensor &_frontRightIR; // Sensor for detecting front-right obstacles

    bool canTurnLeft(float threshold);
    bool canTurnRight(float threshold);
    bool canMoveForward(float threshold);
    bool shouldTurnLeft(int currentHeading, float leftIR, float rightIR, float threshold);
    void alignToWall();
    void moveForward(float distance, int heading, int &currentDistance);
    void initialiseOccupancyGrid();
    void printOccupancyGrid();
    void updateOccupancyGrid(int robotX, int robotY, int heading);
    void markRobotPosition(int robotX, int robotY);
    void markObstacle(int x, int y);
    int convertDistanceToGridBlock(float distance);
    void markFreeSpace(int startX, int startY, int endX, int endY);
    void updateRobotPosition(int &gridX, int &gridY, int heading, int blocksMoved);
};

#endif
