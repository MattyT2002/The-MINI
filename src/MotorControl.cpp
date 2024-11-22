// MotorControlDemo.ino
#include "Arduino.h"
#include "mbed.h"
using namespace mbed;
#include "MotorControl.h"
#include "Encoder.h"


Motor::Motor(PinName PwmPin, PinName dirPin)
    : _PwmPin(PwmPin), _dirPin(dirPin){}

void Motor::setup(void){
    _PwmPin.period(PWM_Period);
    _PwmPin.write(0.0f);
}

void Motor::stop(void){
    _PwmPin.write(0.0f);
}

void Motor::move(int dir, float speed){
    _dirPin.write(dir);
    _PwmPin.write(speed);
}


