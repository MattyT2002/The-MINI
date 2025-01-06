#include "WallFollowing.h"
#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "mbed.h"
#include "rtos.h"
#include "Encoder.h"
#include "IRSensor.h"

using namespace rtos;
using namespace mbed;

int occupancyGrid[GRID_SIZE_X][GRID_SIZE_Y];

WallFollowing::WallFollowing(MovementControl &movement, IR_sensor &leftSideIR, IR_sensor &rightSideIR, IR_sensor &frontLeftIR, IR_sensor &frontRightIR)
    : _movement(movement), _leftSideIR(leftSideIR), _rightSideIR(rightSideIR), _frontLeftIR(frontLeftIR), _frontRightIR(frontRightIR) {}

// Align the robot with the left wall
void WallFollowing::alignToWall()
{
    Serial.println("Aligning to wall...");
    // Use MovementControl's alignToWall method
    _movement.alignToWall();
}

// Check if the robot can turn left
bool WallFollowing::canTurnLeft(float threshold)
{
    float leftDistance = _leftSideIR.read();
    Serial.print("Left Side Sensor: ");
    Serial.println(leftDistance);
    return leftDistance > threshold; // Check if there's enough space on the left
}

bool WallFollowing::canTurnRight(float threshold)
{
    float rightDistance = _rightSideIR.read();
    Serial.print("Right Side Sensor: ");
    Serial.println(rightDistance);
    return rightDistance > threshold; // Check if there's enough space on the left
}

// Check if the robot can move forward
bool WallFollowing::canMoveForward(float threshold)
{
    float frontLeftDistance = _frontLeftIR.read();
    float frontRightDistance = _frontRightIR.read();
    Serial.print("Front Left Sensor: ");
    Serial.println(frontLeftDistance);
    Serial.print("Front Right Sensor: ");
    Serial.println(frontRightDistance);
    return (frontLeftDistance > threshold && frontRightDistance > threshold);
}
bool WallFollowing::shouldTurnLeft(int currentHeading, float leftIR, float rightIR, float threshold)
{
    int leftHeading = (currentHeading - 90) % 360;  // Calculate heading after a left turn
    int rightHeading = (currentHeading + 90) % 360; // Calculate heading after a right turn

    // Debug: Print calculated headings
    Serial.print("Current Heading: ");
    Serial.println(currentHeading);
    Serial.print("Left Heading: ");
    Serial.println(leftHeading);
    Serial.print("Right Heading: ");
    Serial.println(rightHeading);

    // Debug: Print absolute differences
    Serial.print("Abs difference for Left: ");
    Serial.println(abs(leftHeading - 0));
    Serial.print("Abs difference for Right: ");
    Serial.println(abs(rightHeading - 0));

    // Debug: Print IR readings
    Serial.print("Left IR Reading: ");
    Serial.println(leftIR);
    Serial.print("Right IR Reading: ");
    Serial.println(rightIR);

    // Determine turn
    if (abs(leftHeading - 0) < abs(rightHeading - 0))
    {
        Serial.println("Decision: Turn Left (smaller difference)");
        return true; // Turn left
    }
    else if (abs(leftHeading - 0) == abs(rightHeading - 0))
    {
        if (leftIR < threshold)
        { // Wall is close on the left
            Serial.println("Decision: Turn Right (wall close on the left)");
            return false; // Turn right
        }
        else
        { // Ample space on the left
            Serial.println("Decision: Turn Left (ample space on the left)");
            return true; // Turn left
        }
    }
    else
    {
        Serial.println("Decision: Turn Right (smaller difference)");
        return false; // Turn right
    }
}

void WallFollowing::moveForward(float distance, int heading, int &currentDistance) {
    // Move the robot forward
    _movement.forward(distance);

    // Update the distance tracker
    if (heading == 0) {
        // Moving in the desired 0° direction
        currentDistance += distance; 
       
    } else if (heading == 180) {
        // Moving in the opposite direction
        currentDistance -= distance;
    }
}

void WallFollowing::initialiseOccupancyGrid()
{
    for (int x = 0; x < GRID_SIZE_X; x++) {
        for (int y = 0; y < GRID_SIZE_Y; y++) {
            occupancyGrid[x][y] = -1; // Unknown
        }
    }
}

void WallFollowing::printOccupancyGrid()
{
    for (int y = 0; y < GRID_SIZE_Y; y++) {
        for (int x = 0; x < GRID_SIZE_X; x++) {
            if (occupancyGrid[x][y] == -1){
                Serial.print("?");
            } 
            else if (occupancyGrid[x][y] == 1) 
            {
                Serial.print("#");
            }
            else 
            {
                Serial.print(".");
            }
        }
        Serial.println();
    }
}

void WallFollowing::followLeftWall(float setDistance, float moveDistance, int buffer)
{
    int heading = 0;           // Initial heading is 0° (forward direction)
    int failureCounter = 0;    // Counter to track consecutive failures
    const int maxFailures = 3; // Maximum allowed failures before performing a 180 spin
    int totalDistance = 1400;
    int currentDistance = 0;
    initialiseOccupancyGrid();
    printOccupancyGrid();
    
    while (totalDistance > currentDistance)
    {
        Serial.println("----- Begin Loop -----");
        // Read sensor distances
        float frontRight = _frontRightIR.read();
        float frontLeft = _frontLeftIR.read();
        float left = _leftSideIR.read();
        float right = _rightSideIR.read();

        // Log sensor readings for debugging
        Serial.print("Front Right: ");
        Serial.println(frontRight);
        Serial.print("Front Left: ");
        Serial.println(frontLeft);
        Serial.print("Left: ");
        Serial.println(left);
        Serial.print("Right: ");
        Serial.println(right);
        Serial.print("Current Heading: ");
        Serial.println(heading);

        // Detect corner case: front blocked and left wall close
        bool cornerDetectedLeft = (frontRight < moveDistance + buffer && left < moveDistance + buffer * 3);
        bool cornerDetectedRight = (frontRight < moveDistance + buffer && right < moveDistance + buffer * 3);

        if (cornerDetectedLeft)
        {
            Serial.println("Corner detected! Turning right...");
            _movement.turnRight(90);
            heading = (heading + 90) % 360; // Update heading
            failureCounter = 0;             // Reset failure counter
            alignToWall();
            continue; // Skip remaining logic in this loop and reevaluate
        }

        if (cornerDetectedRight)
        {
            Serial.println("Corner detected! Turning left...");
            _movement.turnLeft(90);
            heading = (heading - 90 + 360) % 360; // Update heading
            failureCounter = 0;                   // Reset failure counter
            alignToWall();
            continue; // Skip remaining logic in this loop and reevaluate
        }

        // Normal decision logic
        if (canMoveForward(moveDistance + buffer) && heading == 0)
        {
            Serial.println("Path clear forward. Moving forward...");
            moveForward(moveDistance, heading, currentDistance);
            failureCounter = 0; // Reset failure counter
            alignToWall();
            
        }
        else if (shouldTurnLeft(heading, left, right, (moveDistance + buffer * 3)))
        {
            if (canTurnLeft(moveDistance + buffer * 2))
            {
                if (canMoveForward(moveDistance + buffer))
                {
                    Serial.println("Path clear forward. Moving forward...");
                    moveForward(moveDistance, heading, currentDistance);
                    failureCounter = 0; // Reset failure counter
                    
                }
                Serial.println("Turning left to avoid front wall...");
                _movement.turnLeft(90);
                heading = (heading - 90 + 360) % 360; // Update heading and normalize
                alignToWall();
                failureCounter = 0; // Reset failure counter
            }
            else
            {
                failureCounter++;
            }
        }
        else
        {
            if (canTurnRight(moveDistance + buffer * 3))
            {
                if (canMoveForward(moveDistance + buffer))
                {
                    Serial.println("Path clear forward. Moving forward...");
                    moveForward(moveDistance, heading, currentDistance);
                    failureCounter = 0; // Reset failure counter
                    alignToWall();
                }

                Serial.println("Turning right to avoid front wall...");
                _movement.turnRight(90);
                heading = (heading + 90) % 360; // Update heading and normalize
                alignToWall();
                failureCounter = 0; // Reset failure counter
            }
            else
            {
                failureCounter++;
            }
        }

        // Check if forward movement is possible after turning
        if (canMoveForward(moveDistance + buffer))
        {
            Serial.println("Path clear after turning. Moving forward...");
            moveForward(moveDistance, heading, currentDistance);
            failureCounter = 0; // Reset failure counter
            alignToWall();
        }
        else
        {
            failureCounter++;
        }

        // Check for failure condition
        if (failureCounter >= maxFailures)
        {
            Serial.println("Too many failures! Performing a 180-degree spin...");
            _movement.turnRight(180);
            heading = (heading + 180) % 360; // Update heading for 180-degree turn
            failureCounter = 0;              // Reset failure counter
            alignToWall();
        }

        Serial.println("----- End Loop -----");
    }
}
