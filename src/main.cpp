#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "IRSensor.h"
#include "mbed.h"
using namespace mbed;

IR_sensor IRFrontLeft(FRONT_LEFT);
IR_sensor IRFrontRight(FRONT_RIGHT); 

Motor leftMotor(Left_Motor_PWM, Left_Motor_dir);
Motor rightMotor(Right_Motor_PWM, Right_Motor_dir);
MovementControl movementControl(leftMotor, rightMotor);
void setup()
{
    leftMotor.setup();
    rightMotor.setup();
}
void loop()
{
    int Left = IRFrontLeft.read();
    int Right = IRFrontRight.read();
    Serial.print(Left);
    Serial.println(" left");
    Serial.print(Right);
    Serial.println(" Right");

    /*
    wait_us(5000000);
    movementControl.forward();
    wait_us(5000000);
    movementControl.reverse();
    wait_us(5000000);
    movementControl.stop();
    wait_us(5000000);
    movementControl.turnLeft();
    wait_us(5000000);
    movementControl.turnRight();
    wait_us(5000000);
    */
};