#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "mbed.h"
#include "rtos.h"
#include "Encoder.h"
#include "IRSensor.h"
#include "MazeMapping.h"
#include "ReturnRoute.h"

// create the encoders for each of the motors 
Encoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
Encoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B);

// create the two motors and include there encoder with them
Motor leftMotor(Left_Motor_PWM, Left_Motor_dir, leftEncoder);
Motor rightMotor(Right_Motor_PWM, Right_Motor_dir, rightEncoder);

// create the moement control instances which will be used to 
// control the robots movements and call desired movement behavior
MovementControl movementControl(leftMotor, rightMotor);

// create the side and front inferred sensors
IR_sensor IRSideRight(SIDE_RIGHT);
IR_sensor IRSideLeft(SIDE_LEFT);
IR_sensor IRFrontRight(FRONT_RIGHT);
IR_sensor IRFrontLeft(FRONT_LEFT);

// create a maze mapping behavior used to go through a unknown maze and map it throughout
MazeMapping mazeMap(movementControl, IRSideLeft, IRSideRight, IRFrontLeft, IRFrontRight);
//  creatre a return route behavior which will use the information gathered by a maze map 
//  instances to have an occupancy map to return back through the maze to the start 
ReturnRoute returnRoute(mazeMap, movementControl); 

using namespace rtos;
using namespace mbed;
// movement control constructor combines two motors together to be able to 
// control the movement of the robot as a whole
MovementControl::MovementControl(Motor &leftMotor, Motor &rightMotor)
    : _leftMotor(leftMotor), _rightMotor(rightMotor)
{
    _leftMotor.setup();
    _rightMotor.setup();

    _pwr = Default_pwm;
}

// stops the robot movement by stopping both the motors
void MovementControl::stop()
{
    // stops both the left and right motor
    _leftMotor.stop();
    _rightMotor.stop();
    // reset both the encoder counts to 0 
    leftEncoder.reset();
    rightEncoder.reset();
}

// move the robot forward a given millimetre distances
void MovementControl::forward(float distance)
{
    // resets the encoder to 0 so we know how far we have gone 
    // from now 
    leftEncoder.reset();
    rightEncoder.reset();
    
    // get both the motors to start spinning forward at the same speed
    _leftMotor.move(Left_Forward, Default_pwm);
    _rightMotor.move(Right_Forward, Default_pwm);

    // keep turning the motors until both the encoders have reached the desired distances
    while (leftEncoder.getDistance() < distance && rightEncoder.getDistance() < distance)
    {
        
        // get the current distance the encoders have calculated the wheels have gone
        float leftDist = leftEncoder.getDistance();
        float rightDist = rightEncoder.getDistance();

        // Calculate the difference between the encoder distances readings
        float distanceError = leftDist - rightDist;

        // Apply a small proportional correction to the motors desired PWM to 
        // stop the motor from drifting either direction
        float correction = proportionalResponse * distanceError;
        float leftSpeed = Default_pwm - correction;
        float rightSpeed = Default_pwm + correction;

        // Constrain PWM values to valid range [0.0, 1.0]
        leftSpeed = constrain(leftSpeed, 0.0f, 1.0f);
        rightSpeed = constrain(rightSpeed, 0.0f, 1.0f);
        
        // Update motor with new PWM to deal with drifting
        _leftMotor.move(Left_Forward, leftSpeed);
        _rightMotor.move(Right_Forward, rightSpeed);

        ThisThread::sleep_for(10); // Allow time for adjustment
    }
    // stop motors once desired distanced moved has been reached
    movementControl.stop();
}
// moves the robot backwards a gien millimetre distances
void MovementControl::reverse(float distance)
{
    // resets the encoder to 0 so we know how far we have gone 
    // from now 
    leftEncoder.reset();
    rightEncoder.reset();

    // Start both motors moving backward at the same PWM
    _leftMotor.move(Left_Backwards, Default_pwm);
    _rightMotor.move(Right_Backwards, Default_pwm);

    // keep turning the motors until both the encoders have reached the desired distances
    while (leftEncoder.getDistance() > -distance && rightEncoder.getDistance() > -distance)
    {
        // get the current distance the encoders have calculated the wheels have gone
        float leftDist = leftEncoder.getDistance();
        float rightDist = rightEncoder.getDistance();

        // Calculate the difference between the encoder readings
        float distanceError = leftDist - rightDist;

        // Apply a small proportional correction to the motors desired PWM 
        // to stop drifting 
        float correction = proportionalResponse * distanceError;
        float leftSpeed = Default_pwm + correction;
        float rightSpeed = Default_pwm - correction;

        // Constrain PWM values to valid range [0.0, 1.0]
        leftSpeed = constrain(leftSpeed, 0.0f, 1.0f);
        rightSpeed = constrain(rightSpeed, 0.0f, 1.0f); 
        

        // Update motor speeds to stop drifting
        _leftMotor.move(Left_Backwards, leftSpeed);
        _rightMotor.move(Right_Backwards, rightSpeed);

        ThisThread::sleep_for(10); // Allow time for adjustment
    }

    // Stop both motors after reaching the target distance
    stop();
}

// turn the robot on the spot left (counter clockwise) by a given number of degrees
void MovementControl::turnLeft(int degrees)
{
    // resets the encoder to 0 so we know how far we have gone 
    // from  now
    leftEncoder.reset();
    rightEncoder.reset();

    // Calculate the arc length needed for the turn 
    float arclength = movementControl.getacrlength(degrees);

    // Start both motors in their respective directions and the same PWM
    _leftMotor.move(Left_Forward, Default_pwm);
    _rightMotor.move(Right_Backwards, Default_pwm);

    // keep turning the robot until the wheels have reached the distance of the arc length of the turn
    while (leftEncoder.getDistance() >= -arclength || rightEncoder.getDistance() <= arclength)
    {
        // get the distances the wheels have moved from the encoders
        float leftDist = leftEncoder.getDistance();
        float rightDist = rightEncoder.getDistance();

        // Calculate the difference between the encoder readings
        // Sum since one is negative
        float distanceError = leftDist + rightDist; 

        // Apply a small proportional correction to the motors
        float correction = proportionalResponse * distanceError;
        float leftSpeed = Default_pwm + correction;
        float rightSpeed = Default_pwm - correction;

        // Constrain PWM values to valid range [0.0, 1.0]
        leftSpeed = constrain(leftSpeed, 0.0f, 1.0f);
        rightSpeed = constrain(rightSpeed, 0.0f, 1.0f);


        // Update motor speeds to keep turn on the spot
        _leftMotor.move(Left_Forward, leftSpeed);
        _rightMotor.move(Right_Backwards, rightSpeed);
        
        ThisThread::sleep_for(10); // Allow time for adjustment
    }

    // Stop both motors after completing the turn
    movementControl.stop();
}
// tunr the robot right on the spot right (clockwise) by a given number of degrees 
void MovementControl::turnRight(int degrees)
{
    // resets the encoder to 0 so we know how far we have gone 
    // from now 
    leftEncoder.reset();
    rightEncoder.reset();

    // Calculate the arc length needed for the turn
    float arclength = movementControl.getacrlength(degrees);

    // Start both motors in their respective directions and the same PWM
    _leftMotor.move(Left_Backwards, Default_pwm);
    _rightMotor.move(Right_Forward, Default_pwm);

    // keep turning the robot until the wheels have reached the distance of the arc length of the turn
    while (leftEncoder.getDistance() <= arclength || rightEncoder.getDistance() >= -arclength)
    {
        // get the distances the wheels have moved from the encoders
        float leftDist = leftEncoder.getDistance();
        float rightDist = rightEncoder.getDistance();

        // Calculate the difference between the encoder readings
        // Sum since one is negative
        float distanceError = leftDist + rightDist; 

        // Apply a small proportional correction to the motors
        float correction = proportionalResponse * distanceError;
        float leftSpeed = Default_pwm - correction;
        float rightSpeed = Default_pwm + correction;

        // Constrain PWM values to valid range [0.0, 1.0]
        leftSpeed = constrain(leftSpeed, 0.0f, 1.0f);
        rightSpeed = constrain(rightSpeed, 0.0f, 1.0f);
        

        
        // Update motor speeds to keep turn on the spot
        _leftMotor.move(Left_Backwards, leftSpeed);
        _rightMotor.move(Right_Forward, rightSpeed);

        ThisThread::sleep_for(10); // Allow time for adjustment
    }
    // Stop both motors after completing the turn
    movementControl.stop();
}

// retunr an arc length for a turn on the spot of the robot in a given degrees turn
float MovementControl::getacrlength(float degrees)
{
    return (2 * pi * center_to_wheel) * (degrees / 360);
}

// use the two front sensors to aling the front of the robot with the wall infron to
// keep the robot in line with the walls of the maze
void MovementControl::alignToWall()
{
    
   
    while (true)
    {
        // take a distance reading from the two front sensors
        float leftDistance = IRFrontLeft.read();
        float rightDistance = IRFrontRight.read();
        // calculate the difference between the sensors distances readings
        float distanceError = leftDistance - rightDistance;
        // if the robot is too far away from the wall stop due to sensors not being 
        // accurate enough to do wall alignment and could lead to it getting the robot
        // more out of line with the walls
         if(leftDistance > 200 || rightDistance > 200){
            break;
        }
        
        
        // if the error between the sensors is below the tolerance for aligning 
        // the robot is aligned enough with the wall so end aligning
        if (abs(distanceError) <= tolerance)
        {
            // stop the motors from turning
            movementControl.stop();
            break; // aligment done
        }

        if (distanceError > 0)
        {
            // If left sensor reads further, turn right slightly
            _leftMotor.move(Left_Backwards, alignSpeed);
            _rightMotor.move(Right_Forward, alignSpeed);
        }
        else
        {
            // If right sensor reads further, turn left slightly
            _leftMotor.move(Left_Forward, alignSpeed);
            _rightMotor.move(Right_Backwards, alignSpeed);
        }
        // Add a delay for stability
        ThisThread::sleep_for(10);
    }
}

// call the behaviour to grt the robot to map through the maze and map it as it does so
void MovementControl::mapThroughMaze( float moveDistance, int buffer, int startX, int startY)
{
    // give the behavior a distance it will move each time. a buffer distances it want to be 
    // away from the mazes walls, and the starting position of the robot in the maze
    mazeMap.mapThroughMaze(moveDistance, buffer, startX, startY);
    
}

// return using the a occupancy grid of the maze
void MovementControl::returnUsingMap()
{
    // tell the return route behaviour to uses its map to return through the maze
    returnRoute.ReturnToStart();
}