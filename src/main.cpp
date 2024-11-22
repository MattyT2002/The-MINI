#include "Arduino.h"
#include "MotorControl.h"
#include "MovementControl.h"
#include "IRSensor.h"
#include "mbed.h"
#include "Encoder.h"
using namespace mbed;

IR_sensor IRFrontLeft(FRONT_LEFT);
IR_sensor IRFrontRight(FRONT_RIGHT); 
IR_sensor IRSideRight(SIDE_RIGHT);
IR_sensor IRSideLeft(SIDE_LEFT);

Motor leftMotor(Left_Motor_PWM, Left_Motor_dir);
Motor rightMotor(Right_Motor_PWM, Right_Motor_dir);
MovementControl movementControl(leftMotor, rightMotor);



int count = 0;

rtos::Thread thread;



void setup()
{
    leftMotor.setup();
    rightMotor.setup();
   
   
}
void loop()
{
    
    
   
    /*
    int FrontLeft = IRFrontLeft.read();
    int FrontRight = IRFrontRight.read();
    int SideLeft = IRSideLeft.read();
    int SideRight = IRSideRight.read();
    
    Serial.print("Front left:");
    Serial.println(FrontLeft);
    Serial.print("Front Right:");
    Serial.println(FrontRight);
    
    Serial.print("Side Left:");
    Serial.println(SideLeft);
    Serial.print("Side Right:");
    Serial.println(SideRight);
    
    */
    wait_us(5000000);
    movementControl.forward(100);
    wait_us(3000000);
    movementControl.reverse(100);
    wait_us(3000000);
    movementControl.turnLeft(90);
    wait_us(3000000);
    movementControl.turnRight(90);
   
};