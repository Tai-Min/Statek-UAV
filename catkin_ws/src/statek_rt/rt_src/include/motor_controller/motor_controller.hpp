#pragma once

#include "mbed.h"
#include <ros.h>

#include "../motor/motor.hpp"
#include "../am4096/am4096.hpp"

class MotorController
{
    public:
    struct MotorGpio {
        PinName en;
        PinName cw;
        PinName ccw;
        PinName pwm;    
    };

    enum Side {
        LEFT,
        RIGHT
    };

    enum ControlMode {
        MANUAL,
        AUTO
    };

private:
    // hardware
    Motor motor;
    AM4096 encoder;
    bool reverseEncoder = false;

    // control system stuff
    double setpoint = 0;
    ControlMode controlMode;

    // ros stuff
    Side side;

    // threading stuff
    Thread controlLoopThread;

    void controlLoopThreadFcn();
    
public:
    MotorController(const MotorGpio & motorGPIO, I2C &encoderI2C, uint8_t encoderAddr, Side _side, bool _reverseEncoder = false);
    void start();
    void setVelocity(double sp);
    void setControlMode(ControlMode cm);
};