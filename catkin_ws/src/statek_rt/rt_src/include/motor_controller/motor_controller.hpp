#pragma once

#include <mbed.h>
#include <statek_msgs/Encoder.h>
#include "../ros_handlers/ros.h"
#include "../motor/motor.hpp"
#include "../am4096/am4096.hpp"

class MotorController
{
public:
    /*! Motor control mode. */
    enum ControlMode
    {
        DIRECT,            //!< Setpoint value is scaled by max velocity and passed directly to the motor.
        MAX_VELOCITY_TEST, //!< Sets motor's control signal to maximum value and computes mean speed until test stopped.
        STATE_FEEDBACK     //!< State feedback control.
    };

private:
    // hardware
    Motor motor;                 //!< Motor to control.
    AM4096 encoder;              //!< Encoder attached somehow to the motor.
    double maxVelocity = 0;      //!< Maximum possible velocity. Should be loaded from ROS param server or using setMaxVelocity function.
    bool reverseEncoder = false; //!< Whether inverse encoder's reading.

    double testedMeanVelocity = 0; //!< Mean velocity determined in MAX_VELOCITY_TEST control mode.
    int testedSamplesCounter = 0;  //!< Helper for testedMeanVelocity to compute moving average.

    // control system stuff
    double setpoint = 0;                                   //!< Current setpoint.
    ControlMode controlMode = ControlMode::STATE_FEEDBACK; //!< Current control mode.

    // ros stuff
    statek_msgs::Encoder rawEncoderMsg; //!< Message for raw state reading from encoder.
    ros::Publisher rawEncoderPublisher; //!< Publisher for raw state reading.

    // threading stuff
    Thread controlLoopThread; //!< Thread in which perform motor control.

    /**
     * @brief Function used in controlLoopThread. Loads ROS stuff and performs control algorithms.
     */
    void controlLoopThreadFcn();

    /**
     * @brief Basic saturation.
     * 
     * @param minVal Min possible value.
     * @param maxVal Max possible value.
     * @param val Value to saturate.
     * 
     * @return Saturated value.
     */
    static double saturate(double minVal, double maxVal, double val); //!< Saturate value.

public:
    /**
     * @brief Class contructor.
     */
    MotorController(const Motor::Gpio &motorGpio, I2C &encoderI2c, uint8_t encoderAddr,
                    const char *rawEncoderTopic, const char *tf, bool _reverseEncoder = false);

    /**
     * @brief Start control thread. 
     */
    void start() { this->controlLoopThread.start(callback(this, &MotorController::controlLoopThreadFcn)); }

    /**
     * @brief Sets setpoint velocity in DIRECT and STATE_FEEDBACK control modes.
     * 
     * @param vel Setpoint velocity.
     */
    void setVelocity(double vel) { this->setpoint = this->saturate(-1 * this->maxVelocity, this->maxVelocity, vel); }

    /**
     * @brief Sets control mode.
     * 
     * @param cm Control mode to set.
     */
    void setControlMode(ControlMode cm);

    /**
     * @brief Safety function. Stops the motors instantly.
     */
    void SAFETY_stopMotor();

    /**
     * @brief Returns currently performed control mode.
     * 
     * @return Currently performed control mode.
     */
    ControlMode getControlMode() { return this->controlMode; }

    /**
     * @brief Returns value of testedMeanVelocity. Used to get the result of MAX_VELOCITY_TEST.
     * 
     * @return Tested mean velocity..
     */
    double getTestedMeanVelocity() { return this->testedMeanVelocity; };

    /**
     * @brief Sets max possible velocity. Preferably used after MAX_VELOCITY_TEST to save the result locally.
     * 
     * @param vel Velocity to set as max velocity. 
     */
    void setMaxVelocity(double vel) { this->maxVelocity = vel; }
};