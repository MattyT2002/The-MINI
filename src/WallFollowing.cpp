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
     
    
    while (true)
    {
        Serial.println("----- Begin Loop -----");
        // Align to the left wall if within 15 cm
        float leftSideDistance = _leftSideIR.read();
        Serial.print("Left Side Sensor (Alignment Check): ");
        Serial.println(leftSideDistance);


        // Check if the robot is too close to the wall
        if (isTooCloseToWall(setDistance))
        {
            moveAwayFromWall(moveDistance); // Create space from the wall
        }
        // Check if the robot can turn left
        else if (canMoveForward(setDistance))
        {
            Serial.println("Path clear forward. Moving forward...");
            _movement.forward(moveDistance); // Move forward by the specified distance
            Serial.println("Moved forward.");
        }
        if (canTurnLeft(setDistance))
        {
            if (canMoveForward(setDistance))
            {
                _movement.forward(moveDistance); 
            }
            Serial.println("Space detected on the left. Turning left...");
            _movement.turnLeft(90); // Turn left 90 degrees
            Serial.println("Turned left 90 degrees.");
            alignToWall(); // Align after turning

        
        }
        // Check if the robot can move forward
        
        // If left and forward are blocked, turn right
        else
        {
            Serial.println("Left and forward paths blocked. Turning right...");
            _movement.turnRight(90); // Turn right 90 degrees
            Serial.println("Turned right 90 degrees.");
            alignToWall(); // Align after turning

            // Attempt to move forward after turning right
            if (canMoveForward(setDistance))
            {
                Serial.println("Path clear forward after right turn. Moving forward...");
                _movement.forward(moveDistance);
                Serial.println("Moved forward.");
            }
        }
        Serial.println("----- End Loop -----");
    }
}
