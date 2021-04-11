#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "../include/motor.hpp"
#include "../include/am4096.hpp"
#include "../include/pid.hpp"
#include "../include/low_pass_filter.hpp"

class MotorController
{
public:
    /**
     * @brief Contains error codes that can be returned from MotorController::tryUpdate()
     */
    enum FailCode
    {
        CONTROL_UPDATED,  //!< Success.
        LOOP_RATE_ZERO,   //!< User didn't provided loop rate.
        UPDATE_NOT_READY, //!< Not enough time has passed since previous update.
        ENCODER_FAILURE,  //!< Something gone wrong with the encoder.
        MOTOR_NOT_READY   //!< Ready flag is not set.
    };

    /**
     * @brief One of the possible control modes.
     */
    enum ControlMode
    {
        MAX_VELOCITY_TEST,   //!< Spin with max CV and compute average speed.
        STEP_IDENTIFICATION, //!< Just spin with max CV.
        DIRECT,              //!< Open loop control.
        PID_CONTROL         //!< Closed loop control.
    };

    /**
     * @brief Struct with encoder's state.
     */
    struct EncoderState
    {
        float position;
        float velocity;
        float acceleration;
    };

    /**
     * @brief Params that can be set to this object.
     */
    struct ControlParams
    {
        float kp;
        float ki;
        float kd;
        float smoothingFactor;       //!< Between 0 and 1.
        unsigned int loopUpdateRate; //!< How fast the loop should update in milliseconds.
        float maxVelocity;           //!< In radians per second.
    };

private:
    bool ready = false; //!< Whether the loop should start updating.

    // Hardware
    Motor motor;                 //!< Motor to control.
    AM4096 encoder;              //!< Encoder attached to the motor somehow.
    bool reverseEncoder = false; //!< Whether read encoder as 2PI - reading (if true) or just as reading (if false).

    // For max velocity test control mode.
    float averageVelocity = 0;                   //!< Average velocity computed during MAX_VELOCITY_TEST.
    unsigned int movingAverageSampleTracker = 0; //!< Helper for MotorController::averageVelocity.

    // Control system stuff.
    unsigned long previousUpdateTime = 0;               //!< Timestamp of previous update.
    float setpoint = 0;                                 //!< Setpoint in radians per second.
    float maxVelocity = 0;                              //!< Max possible setpoint.
    unsigned int loopUpdateRate = 30;                   //!< How fast the loop should update in milliseconds.
    ControlMode controlMode = ControlMode::PID_CONTROL; //!< Current control mode.
    LowPassFilter fakeInertia;                          //!< To smooth out the movement of the motor.
    PIDController pid;                                  //!< PID controller.
    EncoderState latestEncoderState;                    //!< Used to compute some stuff in MotorController::update() and to be accessed by anyone interesed.

    /**
     * @brief Read encoder hardware and fill MotorController::EncoderState with readings and estimations.
     * @param ok Set to true on success, false otherwise.
     * @return State of the encoder on success.
     */
    EncoderState getCurrentEncoderState(bool &ok) const;

    /**
     * @brief Get control value that will be written to the motor.
     * This value depends on the control mode but is guaranteed to be between -1 and 1.
     * @param currentEncoderState Current encoder state.
     * @return Control value.
     */
    float getControlValue(const EncoderState &currentEncoderState);

    /**
     * @brief Update control loop.
     * In case this function fails, the setpoint will be set to 0 for safety reasons.
     * @return True if update suceeded. False if there was hardware failure.
     */
    bool update();

    /**
     * @brief Saturate value between min and max.
     * @param minVal Minimum possible value.
     * @param maxVal Maximum possible value
     * @param val Value to saturate.
     * @return Saturated value.
     */
    static float saturate(float minVal, float maxVal, float val);

public:
    /**
     * @brief Class constructor.
     * @param motorGpio GPIO to control the motor.
     * @param encoderAddr I2C address of the encoder.
     * @param _reverseEncoder Whether to reverse encoder reading.
     * @param encoderI2c I2C device to read encoder state on.
     */
    MotorController(const Motor::Gpio &motorGpio, uint8_t encoderAddr, bool _reverseEncoder = false, TwoWire &encoderI2c = Wire);

    /**
     * @brief Start the controller. This will set up some variables.
     */
    void start();

    /**
     * @brief Whether the controller is ready. i.e has all necessary parameters loaded througs MotorController::setMotorParams.
     * @return True if ready.
     */
    bool isReady() const;

    /**
     * @brief Try update the control loop. Can fail if not enough time has passed since last update or due to hardware failure.
     * @return MotorController::FailCode indicating what happened during this function.
     */
    FailCode tryUpdate();

    /**
     * @brief Set setpoint to the controller.
     * @param vel Velocity to request.
     */
    void requestVelocity(float vel);

    /**
     * @brief Set all motor params.
     * 
     * Floats with negative value are ignored.
     * This function sets ready flag if maxVelocity and loopUpdateRate are bigger than 0.
     * Otherwise it resets this flag.
     * @param params Params to set.
     */
    void setMotorParams(const ControlParams &params);

    /**
     * @brief Set control mode.
     * @param cm Control mode to set.
     */
    void setControlMode(ControlMode cm);

    /**
     * @brief Get current setpoint.
     * @return Current setpoint.
     */
    float getRequestedVelocity() const;

    /**
     * @brief Get current control mode.
     * @return current control mode.
     */
    ControlMode getControlMode() const;

    /**
     * @brief Get encoder state from previous update.
     * @return Encoder state.
     */
    EncoderState getLatestEncoderState() const;

    /**
     * @brief Get max possible velocity in radians per second.
     * @return Max possible velocity.
     */
    float getMaxVelocity() const;

    /**
     * @brief Returns average velocity found during MAX_VELOCITY_TEST.
     * @return Result of MAX_VELOCITY_TEST.
     */
    float getMaxVelocityTestResult() const;
};