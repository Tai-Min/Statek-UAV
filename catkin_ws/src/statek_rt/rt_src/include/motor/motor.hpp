#pragma once

#include <mbed.h>

class Motor
{
public:
    /*! GPIO fot the motor */
    struct Gpio
    {
        PinName en;  //!< Enable pin.
        PinName cw;  //!< Clockwise pin.
        PinName ccw; //!< Counter clockwise pin.
        PinName pwm; //!< PWM pin.
    };

private:
    DigitalOut motorEnable;
    DigitalOut motorCW;
    DigitalOut motorCCW;
    PwmOut motorPWM;

public:
    Motor(const Gpio &gpio) : motorEnable(gpio.en), motorCW(gpio.cw), motorCCW(gpio.ccw), motorPWM(gpio.pwm) {}

    void enable()
    {
        motorEnable = true;
    }

    void disable()
    {
        motorEnable = false;
    }

    void write(float dutyCycleSigned)
    {
        bool forward = true;
        if (dutyCycleSigned < 0)
        {
            forward = false;
            dutyCycleSigned *= -1;
        }

        if (forward)
        {
            motorCW = true;
            motorCCW = false;
        }
        else
        {
            motorCW = false;
            motorCCW = true;
        }

        motorPWM = dutyCycleSigned;
    }
};