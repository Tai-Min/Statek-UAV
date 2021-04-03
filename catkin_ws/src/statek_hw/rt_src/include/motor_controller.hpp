#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../include/motor.hpp"
#include "../include/am4096.hpp"
#include "../include/pid.hpp"
#include "../include/low_pass_filter.hpp"

class MotorController {
    public:

    enum FailCode {
        CONTROL_UPDATED,
        LOOP_RATE_ZERO,
        UPDATE_NOT_READY,
        ENCODER_FAILURE
    };

    enum ControlMode
    {
        MAX_VELOCITY_TEST, 
        DIRECT,          
        PID_CONTROL      
    };

    struct EncoderState
    {
        float position;
        float velocity;
        float acceleration;
    };

    struct ControlParams
    {
        float kp;
        float ki;
        float kd;
        unsigned int loopUpdateRate;
        float maxVelocity;
    };

    private:
    // Hardware
    Motor motor;
    AM4096 encoder;
    bool reverseEncoder = false;

    // For max velocity test control mode.
    float averageVelocity = 0;
    unsigned int movingAverageSampleTracker = 0;

    // Control system stuff.
    unsigned long previousUpdateTime = 0;
    float setpoint = 0;
    float maxVelocity = 0;
    unsigned int loopUpdateRate = 0;
    ControlMode controlMode = ControlMode::PID_CONTROL;
    LowPassFilter fakeInertia;
    PIDController pid;
    EncoderState latestEncoderState;

    EncoderState getCurrentEncoderState(bool &ok) const;
    float getControlValue(const EncoderState &currentEncoderState);

    bool update();
    
    static float saturate(float minVal, float maxVal, float val);

    public:
    MotorController(const Motor::Gpio &motorGpio, uint8_t encoderAddr, bool _reverseEncoder = false, TwoWire &encoderI2c = Wire);

    void start();

    FailCode tryUpdate();

    void requestVelocity(float vel);

    void setMotorParams(const ControlParams &params);

    void setControlMode(ControlMode cm);

    float getRequestedVelocity();

    ControlMode getControlMode() const;

    EncoderState getLatestEncoderState() const;

    float getMaxVelocityTestResult() const;
};