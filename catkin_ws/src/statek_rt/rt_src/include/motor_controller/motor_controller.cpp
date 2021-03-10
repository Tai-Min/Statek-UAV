#include "motor_controller.hpp"

#include "../ros_handlers/ros_handlers.hpp"
#include "statek_msgs/Encoder.h"

MotorController::MotorController(const MotorGpio &motorGPIO, I2C &encoderI2C, uint8_t encoderAddr, Side _side, bool _reverseEncoder)
    : motor(motorGPIO.en, motorGPIO.cw, motorGPIO.ccw, motorGPIO.pwm), encoder(encoderI2C, encoderAddr), reverseEncoder(_reverseEncoder), side(_side)
{
}

void MotorController::controlLoopThreadFcn()
{
    uint32_t cntr = 0;
    statek_msgs::Encoder rawEncoderMsg;
    statek_msgs::Encoder filteredEncoderMsg;

    ros::Publisher rawEncoderPublisher(this->side == Side::LEFT ? "motors/left/encoder/raw" : "motors/right/encoder/raw", &rawEncoderMsg);
    nh.advertise(rawEncoderPublisher);
    ros::Publisher filteredEncoderPublisher(this->side == Side::LEFT ? "motors/left/encoder/filtered" : "motors/right/encoder/filtered", &filteredEncoderMsg);
    nh.advertise(filteredEncoderPublisher);

    this->motor.enable();

    while (true)
    {
        // 1. Read.
        double rawVelocity = this->encoder.velocity();
        Thread::wait(1);

        double rawPosition = this->encoder.angle();
        // Inverse encoder's output if necessary.
        if (this->reverseEncoder)
            rawPosition = 2 * M_PI - rawPosition;
        Thread::wait(1);

        // 2. Filter.
        double filteredVelocity = rawVelocity;
        double filteredPosition = rawPosition;

        // 3. Control.
        double controlVal = 0;
        switch (this->controlMode)
        {
        case MANUAL:
            break;
        case VELOCITY_TEST:
            this->testedSamplesCounter++;

            // Simple moving average
            this->testedMeanVelocity = this->testedMeanVelocity +
                                       1 / double(this->testedSamplesCounter) *
                                           (rawVelocity - this->testedMeanVelocity);
            controlVal = 1.0f; // Force motor into max speed for test
            break;
        case AUTO:
            break;
        }

        // 4. Write.
        this->motor.write(controlVal);

        // 5. Publish.
        rawEncoderMsg.header.seq = cntr;
        rawEncoderMsg.header.stamp = nh.now();
        rawEncoderMsg.velocity = rawVelocity;
        rawEncoderMsg.position = rawPosition;
        rawEncoderPublisher.publish(&rawEncoderMsg);

        filteredEncoderMsg.header.seq = cntr;
        filteredEncoderMsg.header.stamp = nh.now();
        filteredEncoderMsg.velocity = filteredVelocity;
        filteredEncoderMsg.position = filteredPosition;
        filteredEncoderPublisher.publish(&filteredEncoderMsg);

        cntr++;
        Thread::wait(48);
    }
}

void MotorController::start()
{
    this->controlLoopThread.start(callback(this, &MotorController::controlLoopThreadFcn));
}

void MotorController::setVelocity(double vel)
{
    this->setpoint = vel;
}

void MotorController::setControlMode(ControlMode cm)
{
    // Reset stuff for velocity test
    if (cm == ControlMode::VELOCITY_TEST)
    {
        this->testedMeanVelocity = 0;
        this->testedSamplesCounter = 0;
    }

    this->controlMode = cm;
}

void MotorController::SAFETY_stopMotor()
{
    this->motor.write(0);
    this->setVelocity(0);
}