#pragma once

#include <Arduino.h>

class Motor
{
public:
    /**
     * @brief GPIO for the motor.
    */
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
    /**
     * @brief Class constructor.
     * @param gpio Motor's GPIO.
     */
    Motor(const Gpio &gpio) : motorEnable(gpio.en), motorCW(gpio.cw), motorCCW(gpio.ccw), motorPWM(gpio.pwm)
    {
        pinMode(motorEnable, OUTPUT);
        pinMode(motorCW, OUTPUT);
        pinMode(motorCCW, OUTPUT);

        // Set up timer (analogWrite works poorly here for some reason). 
        TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(motorPWM), PinMap_PWM);
        pwmChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(motorPWM), PinMap_PWM));

        pwmTimer = new HardwareTimer(instance);
        pwmTimer->setMode(pwmChannel, TIMER_OUTPUT_COMPARE_PWM1, motorPWM);
        pwmTimer->setOverflow(1000, MICROSEC_FORMAT);
        pwmTimer->setCaptureCompare(pwmChannel, 0, PERCENT_COMPARE_FORMAT);
        pwmTimer->resume();
    }

    /**
     * @brief Enable motor.
     */
    void enable() const
    {
        digitalWrite(motorEnable, HIGH);
    }

    /**
     * @brief Disable motor.
     */
    void disable() const
    {
        digitalWrite(motorEnable, LOW);
    }

    /**
     * @brief Set motor speed.
     * @param dutyCycleSigned Value between -1 (max backward) to 1 (max forward).
     */
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
            digitalWrite(motorCW, HIGH);
            digitalWrite(motorCCW, LOW);
        }
        else
        {
            digitalWrite(motorCW, LOW);
            digitalWrite(motorCCW, HIGH);
        }

        if (dutyCycleSigned < 0.1)
            dutyCycleSigned = 0;

        pwmTimer->setCaptureCompare(pwmChannel, 100 * dutyCycleSigned, PERCENT_COMPARE_FORMAT);
    }
};