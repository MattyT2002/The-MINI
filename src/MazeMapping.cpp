#include "MazeMapping.h"
#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "mbed.h"
#include "rtos.h"
#include "Encoder.h"
#include "IRSensor.h"

using namespace rtos;
using namespace mbed;

// maze mapping construtor
MazeMapping::MazeMapping(MovementControl &movement, IR_sensor &leftSideIR, IR_sensor &rightSideIR, IR_sensor &frontLeftIR, IR_sensor &frontRightIR)
    : _movement(movement),
      _leftSideIR(leftSideIR),
      _rightSideIR(rightSideIR),
      _frontLeftIR(frontLeftIR),
      _frontRightIR(frontRightIR) {}

// check if the robot can turn left with threshold being the distance in millimetre required on the left of the robot
// to allow the robot to turn right
bool MazeMapping::canTurnLeft(float threshold, float leftDistance)
{
    // return True if enough distance to turn left
    return leftDistance > threshold;
}

// check if the robot can turn right with threshold being the distance in millimetre required on the right of the robot
// to allow it to turn right
bool MazeMapping::canTurnRight(float threshold, float rightDistance)
{
    // return True if enough distance to turn right
    return rightDistance > threshold; 
}

// Check if the robot can move forward with threshold being the distance in millimetre required infront
// of the robot to allow it to move forward
bool MazeMapping::canMoveForward(float threshold, float frontLeft, float frontRight)
{
    // get a millimetre reading from both the front IR sensors 
    float frontLeftDistance = _frontLeftIR.read();
    float frontRightDistance = _frontRightIR.read();
    // return True if there is enough distance from both the sensor readings to move forward
    return (frontLeftDistance > threshold && frontRightDistance > threshold);
}
// determin if the robot should turn left or right depending on the current heading of the robot 
// and the distance reading from the left and right sensor
bool MazeMapping::shouldTurnLeft(int currentHeading, float leftIR, float rightIR, float threshold)
{
    // calculate the heading after a left and right turn respectively
    int leftHeading = (currentHeading - 90) % 360;  
    int rightHeading = (currentHeading + 90) % 360; 

    // Determine turn 
    // if the left headin is lower so closer to zero we will want to turn left 
    // we absolute the values to deal with negatives
    if (abs(leftHeading) < abs(rightHeading))
    {
        
        return true; // Turn left
    }
    else if (abs(leftHeading - 0) == abs(rightHeading - 0))
    {
        // check if there is enough room to turn left and move 
        if (leftIR < threshold)
        {
            // if cannot turn left then look to turn right
            return false; 
        }
        else
        { // Ample space on the left
            
            return true; // Turn left
        }
    }
    else
    {
        // if right turn will get you close to 0 heading then look to turn right
        return false; 
    }
}

// will make the robot move forward a set distance and updates how far along 
// the robot has made it through the maze
void MazeMapping::moveForward(float distance, int heading, int &currentDistance)
{
    // Move the robot forward a set distance in maltimeters 
    _movement.forward(distance);

    // Update the distance tracker
    if (heading == 0)
    {
        // Moving in the desired 0° direction
        currentDistance += distance;
    }
    else if (heading == 180)
    {
        // Moving in the opposite direction
        currentDistance -= distance;
    }
}

// sets up occupancy grid to have every grid size 
// to be set as -1 representing unknown.
void MazeMapping::initialiseOccupancyGrid()
{
    for (int x = 0; x < GRID_SIZE_X; x++)
    {
        for (int y = 0; y < GRID_SIZE_Y; y++)
        {
            occupancyGrid[x][y] = -1; // Unknown
        }
    }
}

// print the Occupancy Grid for viewing in the terminal
void MazeMapping::printOccupancyGrid()
{
    Serial.println("Occupancy Grid:");
    for (int y = GRID_SIZE_Y - 1; y >= 0; y--) // Start from the top row
    {
        for (int x = 0; x < GRID_SIZE_X; x++)
        {
            if (occupancyGrid[x][y] == -1)
            {
                Serial.print(" ?"); // represents Unknown space
            }
            else if (occupancyGrid[x][y] == 1)
            {
                Serial.print(" #"); // represents Obstacle
            }
            else if (occupancyGrid[x][y] == 2)
            {
                Serial.print("X"); // end position of the robot 
            }
            else if (occupancyGrid[x][y] == -2)
            {
                Serial.print("S"); // starting position of the robot
            }
            else
            {
                Serial.print(" ."); // represents Free space
            }
        }
        Serial.println(); // Move to the next row
    }
}

// mark the robots total foot print on the map based of its current center 
// X, Y coordinates 
void MazeMapping::markRobotPosition(int robotX, int robotY)
{
    // Calculate the bounds of the robot in the grid
    int startX = robotX - 7; 
    int endX = robotX + 7;   
    int startY = robotY - 7;
    int endY = robotY + 7;

    // Ensure the bounds are within the grid limits
    startX = max(0, startX);
    endX = min(GRID_SIZE_X - 1, endX);
    startY = max(0, startY);
    endY = min(GRID_SIZE_Y - 1, endY);

    // Mark all cells within the robot's area as free space
    // on the occupancy grid
    for (int x = startX; x <= endX; x++)
    {
        for (int y = startY; y <= endY; y++)
        {
            occupancyGrid[x][y] = 0; // Free space
        }
    }
}

// update the occupancy grid based of the robots X Y position heading 
// and uses sensor to mark walls in the maze
void MazeMapping::updateOccupancyGrid(int robotX, int robotY, int heading)
{
    // make the current position of the robot onto the grid
    markRobotPosition(robotX, robotY);
    
    // get distance readings in millimetres from all of the sensor  
    float frontLeftDistance = _frontLeftIR.read();
    float frontRightDistance = _frontRightIR.read();
    float leftDistance = _leftSideIR.read();
    float rightDistance = _rightSideIR.read();

    // convert the distance in millimetres from sensors into distance in occupancy grid blocks
    int blocksFromFrountLeftSensorToWall = convertDistanceToGridBlock(frontLeftDistance);
    int blocksFromFrountRightSensorToWall = convertDistanceToGridBlock(frontRightDistance);
    int blocksFromLeftSensorToWall = convertDistanceToGridBlock(leftDistance);
    int blocksFromRightSensorToWall = convertDistanceToGridBlock(rightDistance);

    // the x and y coordinates for the sensors in the grid.
    int frontLeftSensorX;
    int frontLeftSensorY;
    int frontRightSensorX;
    int frontRightSensorY;
    int leftSensorX;
    int leftSensorY;
    int rightSensorX;
    int rightSensorY;
    
    
    /* will use the current heading of the robot
    and robots center location on the grid to calculate
    the location of the sensors on the grid for each heading possible*/
    if (heading == GRID_FORWARD)
    {
        frontLeftSensorX = robotX - 2;
        frontLeftSensorY = robotY + 5;
        frontRightSensorX = robotX + 2;
        frontRightSensorY = robotY + 5;
        leftSensorX = robotX - 4;
        leftSensorY = robotY;
        rightSensorX = robotX + 4;
        rightSensorY = robotY;

        /* mark obstacles based on the sensor readings and the sensor location
        using the heading of the robot to know which direction from the sensor the 
        wall detected was found*/

        // front sensors
        markObstacle(frontLeftSensorX, frontLeftSensorY + blocksFromFrountLeftSensorToWall);
        markObstacle(frontRightSensorX, frontRightSensorY + blocksFromFrountRightSensorToWall);
        // side sensors
        markObstacle(leftSensorX - blocksFromLeftSensorToWall, leftSensorY);
        markObstacle(rightSensorX + blocksFromRightSensorToWall, rightSensorY);
        
        // mark free space between sensor and Wall for each of the sensor readings
        markFreeSpace(frontLeftSensorX, frontLeftSensorY, frontLeftSensorX, frontLeftSensorY + blocksFromFrountLeftSensorToWall);
        markFreeSpace(frontRightSensorX, frontRightSensorY, frontRightSensorX, frontRightSensorY + blocksFromFrountRightSensorToWall);
        markFreeSpace(leftSensorX, leftSensorY, leftSensorX - blocksFromLeftSensorToWall, leftSensorY);
        markFreeSpace(rightSensorX, rightSensorY, rightSensorX + blocksFromRightSensorToWall, rightSensorY);
    }
    else if (heading == GRID_RIGHT)
    {
        frontLeftSensorX = robotX + 5;
        frontLeftSensorY = robotY + 2;
        frontRightSensorX = robotX + 5;
        frontRightSensorY = robotY - 2;
        leftSensorX = robotX;
        leftSensorY = robotY + 4;
        rightSensorX = robotX;
        rightSensorY = robotY - 4;

        /* mark obstacles based on the sensor readings and the sensor location
        using the heading of the robot to know which direction from the sensor the 
        wall detected was found*/

        // front sensors
        markObstacle(frontLeftSensorX + blocksFromFrountLeftSensorToWall, frontLeftSensorY);
        markObstacle(frontRightSensorX + blocksFromFrountRightSensorToWall, frontRightSensorY);
        // side sensors
        markObstacle(leftSensorX, leftSensorY + blocksFromLeftSensorToWall);
        markObstacle(rightSensorX, rightSensorY - blocksFromRightSensorToWall);
        // mark free space between sensor and Wall
        markFreeSpace(frontLeftSensorX, frontLeftSensorY, frontLeftSensorX + blocksFromFrountLeftSensorToWall, frontLeftSensorY);
        markFreeSpace(frontRightSensorX, frontRightSensorY, frontRightSensorX + blocksFromFrountRightSensorToWall, frontRightSensorY);
        markFreeSpace(leftSensorX, leftSensorY, leftSensorX, leftSensorY + blocksFromLeftSensorToWall);
        markFreeSpace(rightSensorX, rightSensorY, rightSensorX, rightSensorY - blocksFromRightSensorToWall);
    }
    else if (heading == GRID_BACKWARDS)
    {
        frontLeftSensorX = robotX + 2;
        frontLeftSensorY = robotY - 5;
        frontRightSensorX = robotX - 2;
        frontRightSensorY = robotY - 5;
        leftSensorX = robotX + 4;
        leftSensorY = robotY;
        rightSensorX = robotX - 4;
        rightSensorY = robotY;

        /* mark obstacles based on the sensor readings and the sensor location
        using the heading of the robot to know which direction from the sensor the 
        wall detected was found*/

        // front sensors
        markObstacle(frontLeftSensorX, frontLeftSensorY - blocksFromFrountLeftSensorToWall);
        markObstacle(frontRightSensorX, frontRightSensorY - blocksFromFrountRightSensorToWall);
        // side sensors
        markObstacle(leftSensorX + blocksFromLeftSensorToWall, leftSensorY);
        markObstacle(rightSensorX - blocksFromRightSensorToWall, rightSensorY);
        // mark free space between sensor and Wall
        markFreeSpace(frontLeftSensorX, frontLeftSensorY, frontLeftSensorX, frontLeftSensorY - blocksFromFrountLeftSensorToWall);
        markFreeSpace(frontRightSensorX, frontRightSensorY, frontRightSensorX, frontRightSensorY - blocksFromFrountRightSensorToWall);
        markFreeSpace(leftSensorX, leftSensorY, leftSensorX + blocksFromLeftSensorToWall, leftSensorY);
        markFreeSpace(rightSensorX, rightSensorY, rightSensorX - blocksFromRightSensorToWall, rightSensorY);
    }
    else if (heading == GRID_LEFT)
    {
        frontLeftSensorX = robotX - 5;
        frontLeftSensorY = robotY - 2;
        frontRightSensorX = robotX - 5;
        frontRightSensorY = robotY + 2;
        leftSensorX = robotX;
        leftSensorY = robotY + 4;
        rightSensorX = robotX;
        rightSensorY = robotY - 4;

        /* mark obstacles based on the sensor readings and the sensor location
        using the heading of the robot to know which direction from the sensor the 
        wall detected was found*/

        // front sensors
        markObstacle(frontLeftSensorX - blocksFromFrountLeftSensorToWall, frontLeftSensorY);
        markObstacle(frontRightSensorX - blocksFromFrountRightSensorToWall, frontRightSensorY);
        // side sensors
        markObstacle(leftSensorX, leftSensorY - blocksFromLeftSensorToWall);
        markObstacle(rightSensorX, rightSensorY + blocksFromRightSensorToWall);
        // mark free space between sensor and Wall
        markFreeSpace(frontLeftSensorX, frontLeftSensorY, frontLeftSensorX - blocksFromFrountLeftSensorToWall, frontLeftSensorY);
        markFreeSpace(frontRightSensorX, frontRightSensorY, frontRightSensorX - blocksFromFrountRightSensorToWall, frontRightSensorY);
        markFreeSpace(leftSensorX, leftSensorY, leftSensorX, leftSensorY - blocksFromLeftSensorToWall);
        markFreeSpace(rightSensorX, rightSensorY, rightSensorX, rightSensorY + blocksFromRightSensorToWall);
    }
}

// mark the blocks on the occupancy grid in a strieght line between 2 
// blocks on the grid
void MazeMapping::markFreeSpace(int startX, int startY, int endX, int endY)
{
    // calculte the change in x and y between the start and end of the free space 
    int dx = abs(endX - startX);
    int dy = abs(endY - startY);
    // figure out which direction you are heading in both x and y axies 
    int sx = (startX < endX) ? 1 : -1;
    int sy = (startY < endY) ? 1 : -1;
    // error term which will be used for smoothing lines 
    int err = 2*(dx - dy);

    while (true)
    {
        // check if current block is the end block
        if (startX == endX && startY == endY)
        {
            // exit due to reaching the end block 
            break;
        }
        // make sure the current block is in the occupancy grid 
        if (startX >= 0 && startX < GRID_SIZE_X && startY >= 0 && startY < GRID_SIZE_Y)
        {
            occupancyGrid[startX][startY] = 0; // Mark as free space
        }
        //create a double error term used for comparision
        int e2 = 2 * err;
        // if error*2 is grater than total -Y direction then we need to move in X
        if (e2 > -dy)
        {
            //adjust error term 
            err -= dy;
            // move one block in the x direction 
            startX += sx; 
        }
        // if total X direction is grater than error*2 them we meed to move in Y
        if (e2 < dx)
        {
            // adjust error term 
            err += dx;
            //move one block in the Y direction
            startY += sy;
        }
    }
}

// mark an obstacle on the grid based off a given X y coordinate for a block 
// in the occupancy grid 
void MazeMapping::markObstacle(int x, int y)
{
    // Ensure the coordinates are within the grid bounds
    if (x >= 0 && x < GRID_SIZE_X && y >= 0 && y < GRID_SIZE_Y)
    {
        occupancyGrid[x][y] = 1; // Mark this cell as occupied 
    }
}
// takes a given millimetre distance and returns a equivelent distances 
// in occupancy grid blocks
int MazeMapping::convertDistanceToGridBlock(float distance)
{
    return round(distance / GRID_CELL_SIZE);
}

// update robot position in the grid based off its previous X Y possision
// heading the robot is moving and the blocks the robot has moved
void MazeMapping::updateRobotPosition(int &gridX, int &gridY, int heading, int blocksMoved)
{
    // Adjust grid coordinates based on heading and blocks moved
    switch (heading)
    {
    case 0: // Moving up
        gridY += blocksMoved;
        break;
    case 90: // Moving right
        gridX += blocksMoved;
        break;
    case 180: // Moving down
        gridY -= blocksMoved;
        break;
    case 270: // Moving left
        gridX -= blocksMoved;
        break;
    }

    // Mark the new position in the occupancy grid
    markRobotPosition(gridX, gridY);
}

/* will navigate through the Maze while recoding an occupancy grid of the maze
given a set move distance for the robot to move each time and a buffer to 
add on to move distance to make a distance needed to been seen by sensors 
to allow movement in that direction and a given starting coordinate for the center of the robot*/
void MazeMapping::MapThroughMaze(float moveDistance, int buffer, int startX, int startY)
{
    // Initial heading is 0° (forward direction)
    int heading = 0;           
     // Counter to track consecutive failures
    int failureCounter = 0;   
    // Maximum allowed failures before performing a 180 spin
    const int maxFailures = 3; 
    // total distance in milimeters the robot needs to move in the 
    // Y directionto reach the end of the maze
    int totalDistance = 1400;
    int currentDistance = 0;
    
    // Robot's position in the occupancy grid
    int gridX = startX; // Initial x-coordinate
    int gridY = startY; // Initial y-coordinate
    //set all the blocks in the occupancy grid to unknown 
    initialiseOccupancyGrid();
    
     // Mark initial robot position
    markRobotPosition(gridX, gridY);

    // keep going through maze solving logic till the robot 
    // has moved enough distance in the Y to have reached the
    // end of the maze
    while (totalDistance > currentDistance)
    {
        // Update occupancy grid with the current robot position
        updateOccupancyGrid(gridX, gridY, heading);
        printOccupancyGrid();
        
        // use front sensors to align front of the robot with the wall
        _movement.alignToWall();
        
        // Read all sensor distances in milimeters
        float frontRight = _frontRightIR.read();
        float frontLeft = _frontLeftIR.read();
        float left = _leftSideIR.read();
        float right = _rightSideIR.read();

        // Detect corner cases
        bool cornerDetectedLeft = (frontRight < moveDistance + buffer && left < moveDistance + buffer * 3);
        bool cornerDetectedRight = (frontRight < moveDistance + buffer && right < moveDistance + buffer * 3);

        if (cornerDetectedLeft)
        {
            Serial.println("Corner detected! Turning right...");
            _movement.turnRight(90);
            heading = (heading + 90) % 360; // Update heading
            failureCounter = 0;
            _movement.alignToWall();
            continue;
        }

        if (cornerDetectedRight)
        {
            Serial.println("Corner detected! Turning left...");
            _movement.turnLeft(90);
            heading = (heading - 90 + 360) % 360; // Update heading
            failureCounter = 0;
            _movement.alignToWall();
            continue;
        }

        // Normal decision logic
        // if the robot has enough room to move forward and is pointing 
        // in the right direction then move forward
        if (canMoveForward(moveDistance + buffer, frontLeft, frontRight) && heading == 0)
        {
            
            moveForward(moveDistance, heading, currentDistance);
            // update the robots position in the occupancy grid based on 
            // the movement that just took place
            updateRobotPosition(gridX, gridY, heading, convertDistanceToGridBlock(moveDistance)); // Update robot position by 1 block
            // reset the failure counter due to moving forward instead of turning
            failureCounter = 0;
            _movement.alignToWall();
        }
        // find out if the robot should move left or right in order 
        // to get around wall or to turn back to 0 heading
        else if (shouldTurnLeft(heading, left, right, (moveDistance + buffer * 3)))
        {
            // if can move forward move forward before turning 
            //to make sure the robot is clear of the wall to the left
            if (canMoveForward(moveDistance + buffer, frontLeft, frontRight))
            {
                // move forward to clear wall
                moveForward(moveDistance, heading, currentDistance);
                // update the robots position in the occupancy grid based on 
                // the movement that just took place
                updateRobotPosition(gridX, gridY, heading, convertDistanceToGridBlock(moveDistance)); // Update robot position by 1 block
                // reset the failure counter due to moving forward instead of turning
                failureCounter = 0;
            _movement.alignToWall();

            // if enough room to the left of the robot 
            if (canTurnLeft(moveDistance + buffer * 2, left))
            {
                // turn left 90 degrees
                _movement.turnLeft(90);
                // update heading
                heading = (heading - 90 + 360) % 360; 
                // reset failure counter
                failureCounter = 0;
                // align to the wall infront
                _movement.alignToWall();
            }
            else
            {
                // iterate falure count
                failureCounter++;
            }
        }
        else
        {
            // see if there is enough room on the right of the robot
            if (canTurnRight(moveDistance + buffer * 3, right))

            {
                 // if can move forward move forward before turning 
                //to make sure the robot is clear of the wall to the right
                if (canMoveForward(moveDistance + buffer, frontLeft, frontRight))
                {
                    // move forward to clear the wall 
                    moveForward(moveDistance, heading, currentDistance);
                    // update the robots position in the occupancy grid based on 
                    // the movement that just took place
                    updateRobotPosition(gridX, gridY, heading, convertDistanceToGridBlock(moveDistance)); // Update robot position by 1 block
                    // reset failure count
                    failureCounter = 0;                                                                   // Reset failure counter
                }

                // turn 90 degrees right 
                _movement.turnRight(90);
                // update heading 
                heading = (heading + 90) % 360; 
                // reset failure count
                failureCounter = 0;
                //align to wall
                _movement.alignToWall();
            }
            else
            {
                // iterate the failure counter due to inability to move
                failureCounter++;
            }
        }

        // Check if forward movement is possible after turning
        if (canMoveForward(moveDistance + buffer, frontLeft, frontRight))
        {
            // move forward
            moveForward(moveDistance, heading, currentDistance);
            // update the robots position on the occupancy grid based off the last movement
            updateRobotPosition(gridX, gridY, heading, convertDistanceToGridBlock(moveDistance)); // Update robot position by 1 block
            failureCounter = 0;
            _movement.alignToWall();
        }
        else
        {
            // iterate the failure counter
            failureCounter++;
        }

        // Check for failure condition
        if (failureCounter >= maxFailures)
        {
            // due to many fails do a 180 turn to avoid getting stuck in a dead end
            _movement.turnRight(180);
            heading = (heading + 180) % 360; // Update heading for 180-degree turn
            // reset failure count 
            failureCounter = 0;
            // align to wall in front
            _movement.alignToWall();
        }

    }

    // mark the end position of the robot
    occupancyGrid[gridX][gridY] = 2;
    // mark the starting position on the map
    // done here to stop it being overridden
    occupancyGrid[startX][startY] = -2;
}
