#ifndef MovementControl_h
#define MotorController_h
#include "Arduino.h"
#include"MotorControl.h"
#include "mbed.h"
using namespace mbed;

#define Right_Forward 0
#define Left_Forward 1
#define Right_Backwards 1
#define Left_Backwards 0

class MovementControl{
    public:
        MovementControl(Motor &leftMotor, Motor &rightMotor);
        void stop();
        void forward();
        void reverse();
        void turnLeft();
        void turnRight();

    private:
        Motor &_leftMotor;
        Motor &_rightMotor;
        float _pwr;
};

extern MovementControl movementControl;


#endif