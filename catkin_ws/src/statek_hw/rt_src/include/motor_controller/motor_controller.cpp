#include "motor_controller.hpp"

#include "../ros_handlers/ros_handlers.hpp"

MotorController::MotorController(const Motor::Gpio &motorGpio, I2C &encoderI2c, uint8_t encoderAddr,
                                 const char *rawEncoderTopic, const char *tf, bool _reverseEncoder)
    : motor(motorGpio),
      encoder(encoderI2c, encoderAddr), reverseEncoder(_reverseEncoder), fakeInertia(0.1),
      rawEncoderPublisher(rawEncoderTopic, &this->rawEncoderMsg)
{
    this->rawEncoderMsg.header.frame_id = tf;
}

void MotorController::controlLoopThreadFcn()
{
    float param;
    while (!nh.getParam("~wheel_max_angular_velocity", &param))
    {
        Thread::wait(5);
    }
    this->maxVelocity = param;

    nh.advertise(this->rawEncoderPublisher);
    nh.spinOnce();

    uint32_t seq = 0;
    this->motor.enable();
    while (true)
    {
        // 1. Read.
        //double rawVelocity = this->encoder.velocity();
        wait_us(20);
        //double rawPosition = this->encoder.position();
        // Inverse encoder's output if necessary.
        //if (this->reverseEncoder)
        //    rawPosition = 2 * M_PI - rawPosition;

        double rawAcceleration = 0;

        // 3. Control.
        double controlVal = 0;
        switch (this->controlMode)
        {
        case DIRECT:
            // Simple proportional control.
            if (this->maxVelocity != 0)
                controlVal = this->setpoint / this->maxVelocity;
            break;
        case MAX_VELOCITY_TEST:
            this->testedSamplesCounter++;
            // Moving average.
            /*this->testedMeanVelocity = this->testedMeanVelocity +
                                       (double)(1 / (double)this->testedSamplesCounter) *
                                           (rawVelocity - this->testedMeanVelocity);*/

            controlVal = 1.0f; // Force motor into max speed for the test.
            break;
        case STATE_FEEDBACK:
            break;
        }

        // 4. Write.
        controlVal = fakeInertia.read(controlVal);
        this->motor.write(controlVal);

        // 5. Publish.
        /*this->rawEncoderMsg.header.seq = seq;
        this->rawEncoderMsg.header.stamp = nh.now();
        this->rawEncoderMsg.velocity = rawVelocity;
        this->rawEncoderMsg.position = rawPosition;
        this->rawEncoderPublisher.publish(&rawEncoderMsg);*/

        seq++;
        Thread::wait(48);
    }
}

double MotorController::saturate(double minVal, double maxVal, double val)
{
    if (val > maxVal)
        val = maxVal;
    if (val < minVal)
        val = minVal;
    return val;
}

void MotorController::setControlMode(ControlMode cm)
{
    // Reset stuff for velocity test
    if (cm == ControlMode::MAX_VELOCITY_TEST)
    {
        this->testedMeanVelocity = 0;
        this->testedSamplesCounter = 0;
    }

    this->controlMode = cm;
}

void MotorController::SAFETY_stopMotor()
{
    //this->motor.write(0);
    this->setVelocity(0);
}