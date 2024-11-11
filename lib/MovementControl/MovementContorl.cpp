#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "mbed.h"
using namespace mbed;

MovementControl::MovementControl(Motor& leftMotor, Motor& rightMotor)
    : _leftMotor(leftMotor), _rightMotor(rightMotor){
        _leftMotor.setup();
        _rightMotor.setup();
        _pwr = Default_pwm;
    }

    void MovementControl::stop(){
        _leftMotor.move(Left_Forward, 0.0f);
        _rightMotor.move(Right_Forward, 0.0f);
    }

    void MovementControl::forward(){
        _leftMotor.move(Left_Forward, _pwr);
        _rightMotor.move(Right_Forward, _pwr);
    }

      void MovementControl::reverse(){
        _leftMotor.move(Left_Backwards, _pwr);
        _rightMotor.move(Right_Backwards, _pwr);
    }

      void MovementControl::turnLeft(){
        _leftMotor.move(Left_Forward, _pwr);
        _rightMotor.move(Right_Backwards, _pwr);
    }
    void MovementControl::turnRight(){
        _leftMotor.move(Left_Backwards, _pwr);
        _rightMotor.move(Right_Forward, _pwr);
    }