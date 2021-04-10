#pragma once

#include <Arduino.h>

class Motor
{
public:
    struct Gpio
    {
        int en;
        int cw;
        int ccw;
        int pwm;
    };

private:
    int motorEnable;
    int motorCW;
    int motorCCW;
    int motorPWM;
    uint32_t pwmChannel;
    HardwareTimer *pwmTimer;

public:
    Motor(const Gpio &gpio) : motorEnable(gpio.en), motorCW(gpio.cw), motorCCW(gpio.ccw), motorPWM(gpio.pwm)
    {
        pinMode(motorEnable, OUTPUT);
        pinMode(motorCW, OUTPUT);
        pinMode(motorCCW, OUTPUT);

        TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(motorPWM), PinMap_PWM);
        pwmChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(motorPWM), PinMap_PWM));

        pwmTimer = new HardwareTimer(instance);
        pwmTimer->setMode(pwmChannel, TIMER_OUTPUT_COMPARE_PWM1, motorPWM);
        pwmTimer->setOverflow(1000, MICROSEC_FORMAT);
        pwmTimer->setCaptureCompare(pwmChannel, 0, PERCENT_COMPARE_FORMAT);
        pwmTimer->resume();
    }

    void enable() const
    {
        digitalWrite(motorEnable, HIGH);
    }

    void disable() const
    {
        digitalWrite(motorEnable, LOW);
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
            digitalWrite(motorCW, HIGH);
            digitalWrite(motorCCW, LOW);
        }
        else
        {
            digitalWrite(motorCW, LOW);
            digitalWrite(motorCCW, HIGH);
        }

        if(dutyCycleSigned < 0.1)
            dutyCycleSigned = 0;
            
        pwmTimer->setCaptureCompare(pwmChannel, 100 * dutyCycleSigned, PERCENT_COMPARE_FORMAT);
    }
};