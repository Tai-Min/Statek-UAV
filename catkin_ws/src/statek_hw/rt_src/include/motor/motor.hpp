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
    DigitalOut motorEnable; //!< Motor enable GPIO.
    DigitalOut motorCW;     //!< Clockwise GPIO.
    DigitalOut motorCCW;    //!< Counter clockwise GPIO.
    PwmOut motorPWM;        //!< PWM GPIO.

public:
    /**
     * @brief Class constructor.
     * 
     * @param gpio Struct with GPIOs required to control the motor.
     */
    Motor(const Gpio &gpio) : motorEnable(gpio.en), motorCW(gpio.cw), motorCCW(gpio.ccw), motorPWM(gpio.pwm) {}

    /**
     * @brief Enable motor.
     */
    void enable()
    {
        motorEnable = true;
    }

    /**
     * @brief Disable motor.
     */
    void disable()
    {
        motorEnable = false;
    }

    /**
     * @brief Control the motor.
     * 
     * @param dutyCycleSigned Value from -1 (max speed in one direction) to 1 (max speed in other direction).
     */
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