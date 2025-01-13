#ifndef MAZEMAPPING_H
#define MAZEMAPPING_H

#include "IRSensor.h"
#include "MovementControl.h"

// How many grid spaces there are on both the X and Y axis of a maze
#define GRID_SIZE_X 145
#define GRID_SIZE_Y 200
// the size of the sides of each grid in the occupancy map in millimetres
#define GRID_CELL_SIZE 10
// Robot size in millimetres (length and width)
#define ROBOT_SIZE 150 
// Robot takes up a 3x3 grid size on the occupancy grid
#define ROBOT_CELLS 15
// Heading representing the direction the robot is facing in relation to the grid
#define GRID_FORWARD 0    
#define GRID_RIGHT 90
#define GRID_BACKWARDS 180
#define GRID_LEFT 270

// Maze mapping is the method used for traversing the maze while recoding an occupancy grid of the maze
class MazeMapping
{
public:
    MazeMapping(MovementControl &movement, IR_sensor &leftSideIR, IR_sensor &rightSideIR, IR_sensor &frontLeftIR, IR_sensor &frontRightIR);
    void mapThroughMaze(float moveDistance, int buffer, int startX, int startY);
    // the occupancyGrids that will be updatesd through the maze
    int occupancyGrid[GRID_SIZE_X][GRID_SIZE_Y];

private:
    // intance of movement control to allow the mazeMapping to move the robot through the maze
    MovementControl &_movement;
    // IR sensor on the left side of the robot
    IR_sensor &_leftSideIR;   
    // IR sensor on the right side of the robot
    IR_sensor &_rightSideIR;  
    // IR sensor at the front left of the robot
    IR_sensor &_frontLeftIR;  
    // IR sensor at the front 
    IR_sensor &_frontRightIR; 
    // return true if there is enough space to the Left of the robot to turn then move
    // using a threshold distance required to turn and with a distances recored by the left sensor
    bool canTurnLeft(float threshold, float leftDistance);
    // return true if there is enough space to the right of the robot to turn then move
    // using a threshold distance required to turn and with a distances recored by the right sensor
    bool canTurnRight(float threshold, float rightDistance);
    // return true if there is enough room for robot to move forwards based of a threshold distances 
    // and a distances sensor reading from both of the front sensors
    bool canMoveForward(float threshold, float frontLeft, float frontRight);
    // return true if based of the current heading and distances reading of the left and right sensors
    // the direction needed to be turn is left and return false if a right turn is required
    bool shouldTurnLeft(int currentHeading, float leftIR, float rightIR, float threshold);
    // moves the robot forward a set distances in milimeters updates how far the robot 
    // has made it through the maze through the Y axis using its current headding to either
    // add subtract the current distance value or keep it same if movement does not effect 
    // Y axis movement
    void moveForward(float distance, int heading, int &currentDistance);
    // sets up Occupancy Grid so all grid squares are represented as unknown areas
    void initialiseOccupancyGrid();
    // prints the occupancy Grid to the terminal in a way the user can interprate
    void printOccupancyGrid();
    // update the occupancy grid based of the robots current position and heading
    // marking the robots current position as free and marking what the sensors 
    // see as walls and the gap between sensor and wall as free space
    void updateOccupancyGrid(int robotX, int robotY, int heading);
    // marks the robots current position in the maze onto the occupancy grind as free space
    void markRobotPosition(int robotX, int robotY);
    // mark an obstacle in the occupancy grid given a X Y coordinate value of the location
    // the obstacle was detected
    void markObstacle(int x, int y);
    // return an equivalent distance in grid block measurements for a given distances in milimeters
    int convertDistanceToGridBlock(float distance);
    // mark the distance between two squares on the occupancy grid as free space
    void markFreeSpace(int startX, int startY, int endX, int endY);
    // update the robots new position in the maze based of the current position of the robot 
    // with heading it is moving and how many blocks it moved in that direction
    void updateRobotPosition(int &gridX, int &gridY, int heading, int blocksMoved);
};

#endif
