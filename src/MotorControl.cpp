// MotorControlDemo.ino
#include "Arduino.h"
#include "mbed.h"
using namespace mbed;
#include "MotorControl.h"
#include "Encoder.h"


Encoder _encoder; 

Motor::Motor(PinName PwmPin, PinName dirPin)
    : _PwmPin(PwmPin), _dirPin(dirPin), _kp(kp), _ki(ki), _kd(kd), _prevError(0), _integral(0), _targetVel(0), _currentVel(0){}

void Motor::setup(void){
    _PwmPin.period(PWM_Period);
    _PwmPin.write(0.0f);
    _PIDTicker.attach(callback(this, &Motor::PID),0.1);
    _velTicker.attach(callback(this, &Motor::calCurrentVel),0.1);
}

void Motor::stop(void){
    _PwmPin.write(0.0f);
}

void Motor::move(int dir, float speed){
    _dirPin.write(dir);
    _PwmPin.write(speed);
}

void Motor::PID(){
    float error = _targetVel -_currentVel;

    float Pout = _kp * error;

    _integral += error * 0.1f;

    float Iout = _ki * _integral;

    float derivative = (error - _prevError) / 0.1f;
    float Dout = kd * derivative;

    float output = Pout + Iout + Dout;

    output = max(0.0f, min(1.0f, output));

    _PwmPin.write(output);

    _prevError = error;
} 

void Motor::calCurrentVel(){
    if(!_encoder) return;
    _currentVel = _encoder->getDistance()/0.1f;
    _encoder->reset();
}

void Motor::setMotorVel(float targetVel){
    _targetVel = targetVel;
}