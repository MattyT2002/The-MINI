#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "mbed.h"
#include "rtos.h"
#include "Encoder.h"

Encoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B); 
Encoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B);

Motor leftMotor(Left_Motor_PWM, Left_Motor_dir, leftEncoder);
Motor rightMotor(Right_Motor_PWM, Right_Motor_dir, rightEncoder);

MovementControl movementControl(leftMotor, rightMotor);
 
using namespace rtos;
using namespace mbed;

MovementControl::MovementControl(Motor &leftMotor, Motor &rightMotor)
    : _leftMotor(leftMotor), _rightMotor(rightMotor){
        _leftMotor.setup();
        _rightMotor.setup();
        
        _pwr = Default_pwm;
    }

    

    void MovementControl::stop(){
        Thread leftWheelThread, RightWheelThread;
        leftWheelThread.start(callback([&](){
            _leftMotor.stop();
        }));
        
        RightWheelThread.start(callback([&](){
            _rightMotor.stop();
        }));
        RightWheelThread.join();
        leftWheelThread.join();
        _leftMotor.setMotorVel(0.0f);  // Set desired velocity for left motor
        _rightMotor.setMotorVel(0.0f);
        leftEncoder.reset();
        rightEncoder.reset();
       
    }

    void MovementControl::forward(float distance){
       leftEncoder.reset();
    rightEncoder.reset();

    

    _leftMotor.move(Left_Forward, Default_pwm);
    _rightMotor.move(Right_Forward, Default_pwm);

    while (leftEncoder.getDistance() < distance && rightEncoder.getDistance() < distance) {
        float leftDist = leftEncoder.getDistance();
        float rightDist = rightEncoder.getDistance();

        // Calculate the difference between the encoder readings
        float distanceError = leftDist - rightDist;

        // Apply a small proportional correction to the motors
        float correction = 0.01f * distanceError; // Tunable factor (e.g., 0.01)
        float leftSpeed = Default_pwm - correction;
        float rightSpeed = Default_pwm + correction;

        // Constrain PWM values to valid range [0.0, 1.0]
        leftSpeed = constrain(leftSpeed, 0.0f, 1.0f);
        rightSpeed = constrain(rightSpeed, 0.0f, 1.0f);
        Serial.println(leftSpeed);
        // Update motor speeds
        _leftMotor.move(Left_Forward, leftSpeed);
        _rightMotor.move(Right_Forward, rightSpeed);

        // Debugging information
        Serial.print("Left Dist: ");
        Serial.print(leftDist);
        Serial.print(" | Right Dist: ");
        Serial.print(rightDist);
        Serial.print(" | Correction: ");
        Serial.println(correction);

        ThisThread::sleep_for(10); // Allow time for adjustment
    }
        movementControl.stop();
}
    void MovementControl::reverse(float distance) {
    leftEncoder.reset();
    rightEncoder.reset();

    // Start both motors moving backward
    _leftMotor.move(Left_Backwards, Default_pwm);
    _rightMotor.move(Right_Backwards, Default_pwm);

    while (leftEncoder.getDistance() > -distance && rightEncoder.getDistance() > -distance) {
        float leftDist = leftEncoder.getDistance();
        float rightDist = rightEncoder.getDistance();

        // Calculate the difference between the encoder readings
        float distanceError = leftDist - rightDist;

        // Apply a small proportional correction to the motors
        float correction = 0.01f * distanceError; // Tunable factor (e.g., 0.01)
        float leftSpeed = Default_pwm + correction;
        float rightSpeed = Default_pwm - correction;

        // Constrain PWM values to valid range [0.0, 1.0]
        leftSpeed = constrain(leftSpeed, 0.0f, 1.0f);
        rightSpeed = constrain(rightSpeed, 0.0f, 1.0f);

        // Debugging information
        Serial.print("Left Dist: ");
        Serial.print(leftDist);
        Serial.print(" | Right Dist: ");
        Serial.print(rightDist);
        Serial.print(" | Correction: ");
        Serial.println(correction);

        // Update motor speeds
        _leftMotor.move(Left_Backwards, leftSpeed);
        _rightMotor.move(Right_Backwards, rightSpeed);

        ThisThread::sleep_for(10); // Allow time for adjustment
    }

    // Stop both motors after reaching the target distance
    stop();
}

      
      void MovementControl::turnLeft(int degrees) {
    leftEncoder.reset();
    rightEncoder.reset();

    // Calculate the arc length needed for the turn
    float arclength = movementControl.getacrlength(degrees);

    // Start both motors in their respective directions
    _leftMotor.move(Left_Forward, Default_pwm);
    _rightMotor.move(Right_Backwards, Default_pwm);

    while (leftEncoder.getDistance() > -arclength && rightEncoder.getDistance() < arclength) {
        float leftDist = leftEncoder.getDistance();
        float rightDist = rightEncoder.getDistance();

        // Calculate the difference between the encoder readings
        float distanceError = leftDist + rightDist; // Sum since one is negative

        // Apply a small proportional correction to the motors
        float correction = 0.01f * distanceError; // Tunable factor (e.g., 0.01)
        float leftSpeed = Default_pwm - correction;
        float rightSpeed = Default_pwm + correction;

        // Constrain PWM values to valid range [0.0, 1.0]
        leftSpeed = constrain(leftSpeed, 0.0f, 1.0f);
        rightSpeed = constrain(rightSpeed, 0.0f, 1.0f);

        // Debugging information
        Serial.print("Left Dist: ");
        Serial.print(leftDist);
        Serial.print(" | Right Dist: ");
        Serial.print(rightDist);
        Serial.print(" | Correction: ");
        Serial.println(correction);

        // Update motor speeds
        _leftMotor.move(Left_Forward, leftSpeed);
        _rightMotor.move(Right_Backwards, rightSpeed);

        ThisThread::sleep_for(10); // Allow time for adjustment
    }

    // Stop both motors after completing the turn
    movementControl.stop();
}

    void MovementControl::turnRight(int degrees){
         Thread leftWheelThread, RightWheelThread;
        
        leftWheelThread.start(callback([&](){
            _leftMotor.move(Left_Backwards, _pwr);
        }));

        RightWheelThread.start(callback([&](){
            _rightMotor.move(Right_Forward, _pwr);
        }));
        RightWheelThread.join();
        leftWheelThread.join();
        float arclength = movementControl.getacrlength(degrees);
        Serial.println(arclength);
        while (leftEncoder.getDistance() <= arclength && rightEncoder.getDistance() >= -arclength )
        {
            Serial.print("Left: ");
            Serial.print(leftEncoder.getDistance());
            Serial.print(" | Right: ");
            Serial.println(rightEncoder.getDistance());
        }
        movementControl.stop();
    }

    float MovementControl::getacrlength(float degrees){
        return (2*pi*center_to_wheel)*(degrees/360);
    }

    void MovementControl::setVelocity(int velocity){
        _leftMotor.setMotorVel(velocity);
        _rightMotor.setMotorVel(velocity);
    }

    void MovementControl::forwardTest(){
        
        leftMotor.move(Left_Forward,0.5);
        rightMotor.move(Right_Forward,0.5);
    }

   