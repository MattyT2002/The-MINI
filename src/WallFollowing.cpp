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
        else if (frontLeft < setDistance || frontRight < setDistance) // If front wall detected
        {
            // Determine which turn (left or right) will get closer to heading 0°
            if (shouldTurnLeft(heading))
            {
                Serial.println("Prioritizing left turn to get closer to initial heading...");
                _movement.turnLeft(90);
                heading = (heading - 90 + 360) % 360; // Update heading
                alignToWall();
            }
            else
            {
                Serial.println("Prioritizing right turn to get closer to initial heading...");
                _movement.turnRight(90);
                heading = (heading + 90) % 360; // Update heading
                alignToWall();
            }
        }
        // Check for left or right openings if no immediate obstacles in front
        else if (left > setDistance)
        {
            Serial.println("Left opening detected. Turning left...");
            _movement.turnLeft(90);
            heading = (heading - 90 + 360) % 360; // Update heading
            alignToWall();
        }
        else if (right > setDistance)
        {
            Serial.println("Right opening detected. Turning right...");
            _movement.turnRight(90);
            heading = (heading + 90) % 360; // Update heading
            alignToWall();
        }
        else
        {
            Serial.println("No valid movement detected. Stopping.");
        }

        Serial.println("----- End Loop -----");
    }
}

// Helper function to decide if a left turn is closer to heading 0°
bool WallFollowing::shouldTurnLeft(int currentHeading)
{
    int leftHeading = (currentHeading - 90 + 360) % 360; // Calculate heading after a left turn
    int rightHeading = (currentHeading + 90) % 360;      // Calculate heading after a right turn

    // Compare absolute differences to determine the closer turn
    return abs(leftHeading - 0) < abs(rightHeading - 0);
}

