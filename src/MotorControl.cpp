// MotorControlDemo.ino
#include "Arduino.h"
#include "mbed.h"
using namespace mbed;
#include "MotorControl.h"
#include "Encoder.h"


Motor::Motor(PinName PwmPin, PinName dirPin, Encoder &encoder)
    : _PwmPin(PwmPin), _dirPin(dirPin), _prevError(0), _integral(0), _targetVel(0), _currentVel(0),_encoder(encoder){}

void Motor::setup(void){
    _encoder.setup();
    _PwmPin.period(PWM_Period);
    _PwmPin.write(0.0f);
    //_PIDTicker.attach(callback(this, &Motor::PID),0.1f);
    _velTicker.attach(callback(this, &Motor::calCurrentVel),0.1f);
}

void Motor::stop(void){
    _PwmPin.write(0.0f);
}

void Motor::move(int dir, float speed){
    _dirPin.write(dir);
    _PwmPin.write(speed);
}

void Motor::PID(){
    float error = _targetVel - _currentVel;  // Calculate the velocity error
    float Pout = _kp * error;               // Proportional output
    float output = constrain(Pout, 0.0f, 1.0f); // Ensure PWM is within [0, 1]

    Serial.print("TargetVel: ");
    Serial.print(_targetVel);
    Serial.print(" | CurrentVel: ");
    Serial.print(_currentVel);
    Serial.print(" | Output: ");
    Serial.println(output);

    _PwmPin.write(output);                  // Update motor PWM
    _prevError = error;                     // Update previous error
} 

void Motor::calCurrentVel(){
    float currentDist = _encoder.getDistance();
    float incrementalDist = currentDist - _prevEncoderDist;
    _currentVel = (incrementalDist/0.1f);
    _prevEncoderDist = currentDist;
}

void Motor::setMotorVel(float targetVel){
    _targetVel = targetVel;
}