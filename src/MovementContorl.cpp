#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "mbed.h"
#include "rtos.h"
#include "Encoder.h"

Encoder rightEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B); 
Encoder leftEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B);


using namespace rtos;
using namespace mbed;

MovementControl::MovementControl(Motor& leftMotor, Motor& rightMotor)
    : _leftMotor(leftMotor), _rightMotor(rightMotor){
        _leftMotor.setup();
        _rightMotor.setup();
        rightEncoder.setup();
        leftEncoder.setup();
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
        leftEncoder.reset();
        rightEncoder.reset();
       
    }

    void MovementControl::forward(float distance){
        Thread leftWheelThread, RightWheelThread;
        
        leftWheelThread.start(callback([&](){
            _leftMotor.move(Left_Forward, _pwr);
        }));

        RightWheelThread.start(callback([&](){
            _rightMotor.move(Right_Forward, _pwr);
        }));
        leftWheelThread.join();
        RightWheelThread.join();

        while (leftEncoder.getDistance() <= distance && rightEncoder.getDistance() <= distance )
        {
            Serial.print("Left: ");
            Serial.print(leftEncoder.getDistance());
            Serial.print(" | Right: ");
            Serial.println(rightEncoder.getDistance());
        }
        movementControl.stop();
        
       
    }

      void MovementControl::reverse(float distance){
      Thread leftWheelThread, RightWheelThread;
        
        leftWheelThread.start(callback([&](){
            _leftMotor.move(Left_Backwards, _pwr);
        }));

        RightWheelThread.start(callback([&](){
            _rightMotor.move(Right_Backwards, _pwr);
        }));
        RightWheelThread.join();
        leftWheelThread.join();
        
         while (leftEncoder.getDistance() >= -distance && rightEncoder.getDistance() >= -distance )
        {
            Serial.print("Left: ");
            Serial.print(leftEncoder.getDistance());
            Serial.print(" | Right: ");
            Serial.println(rightEncoder.getDistance());
        }
        
        movementControl.stop();
    }

      void MovementControl::turnLeft(int degrees){
        Thread leftWheelThread, RightWheelThread;
        
        leftWheelThread.start(callback([&](){
            _leftMotor.move(Left_Forward, _pwr);
        }));

        RightWheelThread.start(callback([&](){
            _rightMotor.move(Right_Backwards, _pwr);
        }));
        RightWheelThread.join();
        leftWheelThread.join();
        float arclength = movementControl.getacrlength(degrees);
         while (leftEncoder.getDistance() >= -arclength && rightEncoder.getDistance() <= arclength )
        {
            Serial.print("Left: ");
            Serial.print(leftEncoder.getDistance());
            Serial.print(" | Right: ");
            Serial.println(rightEncoder.getDistance());
        }
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