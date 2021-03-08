#pragma once

#include "mbed.h"

class Motor
{
private:
    DigitalOut motorEnable;
    DigitalOut motorCW;
    DigitalOut motorCCW;
    PwmOut motorPWM;

public:
    Motor(PinName en, PinName cw, PinName ccw, PinName pwm) : motorEnable(en), motorCW(cw), motorCCW(ccw), motorPWM(pwm) {}

    void enable(){
        motorEnable = true;
    }

    void disable(){
        motorEnable = false;
    }

    void write(float dutyCycleSigned)
    {
        bool forward = true;
        if(dutyCycleSigned < 0){
            forward = false;
            dutyCycleSigned *= -1;
        }

        if(forward){
            motorCW = true;
            motorCCW = false;
        }
        else{
            motorCW = false;
            motorCCW = true;
        }

        motorPWM = dutyCycleSigned;
    }
};