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

// Check if the robot is too close to the left wall
bool WallFollowing::isTooCloseToWall(float setDistance)
{
    float leftDistance = _leftSideIR.read();
    Serial.print("Left Side Sensor (Check for too close): ");
    Serial.println(leftDistance);
    return leftDistance < setDistance;
}

// Move away from the wall if too close
void WallFollowing::moveAwayFromWall(float moveDistance)
{
    Serial.println("Too close to wall. Moving away...");
    // Turn 90 degrees to move away from the wall
    _movement.turnRight(90);
    Serial.println("Turned right 90 degrees.");
    _movement.forward(moveDistance);
    Serial.println("Moved forward to create space.");
    // Realign with the wall after moving away
    alignToWall();
}

// Main wall-following logic
void WallFollowing::followLeftWall(float setDistance, float moveDistance)
{
    int heading = 0; // Initial heading is 0° (forward direction)

    while (true)
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

        // Prioritize moving forward in the initial direction
        if (canMoveForward(moveDistance) && heading == 0)
        {
            Serial.println("Path clear forward and maintaining initial heading. Moving forward...");
            _movement.forward(moveDistance);
        }
        // Check for left opening if forward is blocked
        else if (left > setDistance)
        {
            if(canMoveForward(moveDistance/2)){
                _movement.forward(moveDistance/2);
            }
            Serial.println("Left opening detected. Turning left...");
            _movement.turnLeft(90);
            heading = (heading - 90 + 360) % 360; // Update heading (keep in range 0-359)
            alignToWall();
        }
        // Check for front wall
        else if (frontLeft < setDistance || frontRight < setDistance)
        {
            // Check for right opening
            if (right > setDistance)
            {
                Serial.println("Right opening detected. Turning right...");
                _movement.turnRight(90);
                heading = (heading + 90) % 360; // Update heading
                alignToWall();
            }
            else
            {
                Serial.println("No openings detected. Turning around...");
                _movement.turnRight(180);
                heading = (heading + 180) % 360; // Update heading
                alignToWall();
            }
        }
        // If no walls or openings are detected, move forward
        else if (canMoveForward(moveDistance))
        {
            Serial.println("Path clear forward. Moving forward...");
            _movement.forward(moveDistance);
        }
        else
        {
            Serial.println("No valid movement detected. Stopping.");
        }

        Serial.println("----- End Loop -----");
    }
}
