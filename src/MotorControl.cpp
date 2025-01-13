// MotorControlDemo.ino
#include "Arduino.h"
#include "mbed.h"
using namespace mbed;
#include "MotorControl.h"
#include "Encoder.h"

// motor constructor and will initialise the pins to control
// the PWM and direction of the motor and the encoder connected to the motor
Motor::Motor(PinName PwmPin, PinName dirPin, Encoder &encoder)
    : _PwmPin(PwmPin), 
      _dirPin(dirPin), 
      _encoder(encoder) {}

// function for seting up the motor
void Motor::setup(void)
{
    // sets up the encoder linked to the motor 
    _encoder.setup();
    // set the frequency of the of the PWM signal
    _PwmPin.period(PWM_Period);
    // sets PWM duty cycle to 0 
    _PwmPin.write(0.0f);
    
}

// function to make the motor stop turning
void Motor::stop(void)
{
    // sets PWM duty cycle to 0 
    _PwmPin.write(0.0f);
}

// sets the motors rotation direction and the duty cycle of the PWM to control speed
void Motor::move(int dir, float speed)
{
    // write the direction you want to the motor direction pin
    _dirPin.write(dir);
    // write the PWM duty cycle to the motors PWM pin to control the speed
    _PwmPin.write(speed);
}



