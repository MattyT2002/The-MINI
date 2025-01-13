// MotorControl.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "mbed.h"
#include "Encoder.h"

// the pulse width modulation period for the signal sent to the motors
#define PWM_Period 0.01f
// the pin values for where each of the motors pulse width modulation pin 
// and there direction pins will be connected
#define Left_Motor_PWM P1_2
#define Left_Motor_dir P0_4
#define Right_Motor_PWM P0_27
#define Right_Motor_dir P0_5
// the standard pulse width modulation duty cycle that will be used when moving 
// the motor
#define Default_pwm 0.4f

using namespace mbed;
class Motor
{
public:
  // motor contructor giving the motor a pin to change its Pulse width modulation
  // a pin to change its direction and a encoder to monitor the wheels distances moved
  Motor(PinName PwmPin, PinName dirPin, Encoder &encoder);
  // sets up the motor with the encoder and sets the PWM frequency 
  void setup();
  // stops the motor from spinning
  void stop();
  // sets the motor direction for turning and a PWM duty cycle which will 
  // control its starting speed
  void move(int dir, float speed);

private:
  // define what the encoders and pins are
  Encoder &_encoder;
  PwmOut _PwmPin;
  DigitalOut _dirPin;
 
};
extern Motor leftMotor;
extern Motor rightMotor;

#endif
