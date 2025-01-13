// MotorControl.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "mbed.h"
#include "Encoder.h"

#define PWM_Period 0.01f
#define Left_Motor_PWM P1_2
#define Left_Motor_dir P0_4
#define Right_Motor_PWM P0_27
#define Right_Motor_dir P0_5
#define Default_pwm 0.3f

using namespace mbed;
class Motor
{
public:
  Motor(PinName PwmPin, PinName dirPin, Encoder &encoder);
  void setup();
  void stop();
  void move(int dir, float speed);

private:
  Encoder &_encoder;
  PwmOut _PwmPin;
  DigitalOut _dirPin;
 
};
extern Motor leftMotor;
extern Motor rightMotor;

#endif
