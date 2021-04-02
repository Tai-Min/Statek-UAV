#pragma once

#include <Arduino.h>

class Motor
{
public:
    struct Gpio
    {
        PinName en;
        PinName cw;
        PinName ccw;
        PinName pwm;
    };

private:
    PinName motorEnable;
    PinName motorCW;
    PinName motorCCW;
    PinName motorPWM;

public:

    Motor(const Gpio &gpio) : motorEnable(gpio.en), motorCW(gpio.cw), motorCCW(gpio.ccw), motorPWM(gpio.pwm) {
        pinMode(motorEnable, OUTPUT);
        pinMode(motorCW, OUTPUT);
        pinMode(motorCCW, OUTPUT);
        pinMode(motorPWM, OUTPUT);
    }

    void enable() const
    {
        digitalWrite(motorEnable, true);
    }

    void disable() const
    {
        digitalWrite(motorEnable, true);
    }

    void write(float dutyCycleSigned) const
    {
        bool forward = true;
        if (dutyCycleSigned < 0)
        {
            forward = false;
            dutyCycleSigned *= -1;
        }

        if (forward)
        {
            digitalWrite(motorCW, true);
            digitalWrite(motorCCW, false);
        }
        else
        {
            digitalWrite(motorCW, false);
            digitalWrite(motorCCW, true);
        }

        analogWrite(motorPWM, dutyCycleSigned * 255);
    }
};