#include "motor_controller.hpp"

#include "../ros_handlers/ros_handlers.hpp"
#include "../clock/clock.hpp"

MotorController::MotorController(const Motor::Gpio &motorGpio, I2C &encoderI2c, uint8_t encoderAddr,
                                 const char *rawEncoderTopic, const char *pidKp, const char *pidKi,
                                 const char *pidKd, const char *tf, bool _reverseEncoder)
    : motor(motorGpio), encoder(encoderI2c, encoderAddr), reverseEncoder(_reverseEncoder),
      fakeInertia(0.1), pid(-1, 1), rawEncoderPublisher(rawEncoderTopic, &this->rawEncoderMsg)
{
    // Save tf link.
    this->rawEncoderMsg.header.frame_id = tf;

    // Get params.
    float maxVelocity;
    while (!nh.getParam("~wheel_max_angular_velocity", &maxVelocity))
    {
        Thread::wait(5);
        nh.spinOnce();
    }
    this->maxVelocity = maxVelocity;

    int loopUpdateRate;
    while (!nh.getParam("~loop_rate_ms", &loopUpdateRate))
    {
        Thread::wait(5);
        nh.spinOnce();
    }
    this->loopUpdateRate = loopUpdateRate;
    pid.setSamplingTime(loopUpdateRate / 1000.0);

    int encoderPublishRate;
    while (!nh.getParam("~encoder_publish_rate_ms", &encoderPublishRate))
    {
        Thread::wait(5);
        nh.spinOnce();
    }
    this->encoderPublishRate = encoderPublishRate;

    float kp;
    while (!nh.getParam(pidKp, &kp))
    {
        Thread::wait(5);
        nh.spinOnce();
    }
    this->pid.setKp(kp);

    float ki;
    while (!nh.getParam(pidKi, &ki))
    {
        Thread::wait(5);
        nh.spinOnce();
    }
    this->pid.setKi(ki);

    float kd;
    while (!nh.getParam(pidKd, &kd))
    {
        Thread::wait(5);
        nh.spinOnce();
    }
    this->pid.setKd(kd);
}

void MotorController::controlLoopThreadFcn()
{
    this->loopUpdateRate = 30;
    this->encoderPublishRate = 100;

    // Advertise encoder topic
    nh.advertise(this->rawEncoderPublisher);
    nh.spinOnce();

    // Prepare some helper variables
    uint32_t seq = 0;
    unsigned long previousUpdateTime = clockNow();
    unsigned long previousEncoderPublishTime = clockNow();
    double previousPosition = 0;
    double previousVelocity = 0;
    const double loopUpdateRateInSeconds = (double)(this->loopUpdateRate / 1000.0);

    this->motor.enable();
    while (true)
    {
        // 1. Read.
        double currentPosition = this->encoder.position();
        // Inverse encoder's output if necessary.
        if (this->reverseEncoder)
            currentPosition = 2 * M_PI - currentPosition;

        // Handle position overflow while moving forward
        if (previousPosition > 5.5 && currentPosition < 1)
        {
            previousPosition = previousPosition - 2 * M_PI;
        }
        // Handle position overflow while moving backward
        else if (previousPosition < 1 && currentPosition > 5.5)
        {
            previousPosition = previousPosition + 2 * M_PI;
        }

        double currentVelocity = this->encoder.velocity();

        // Find the sign of the velocity
        if (currentPosition < previousPosition)
        {
            currentVelocity *= -1;
        }

        // Use velocity and position to estimate acceleration
        // Compute simple mean of estimations
        // Maybe add Kalman filter later?
        double accelerationFromPosition = currentPosition - previousPosition - previousVelocity * loopUpdateRateInSeconds;
        double accelerationFromVelocity = (currentVelocity - previousVelocity) * 1 / loopUpdateRateInSeconds;
        double currentAcceleration = (accelerationFromPosition + accelerationFromVelocity) / 2.0;

        // 2. Control.
        double controlVal = 0;
        switch (this->controlMode)
        {
        case MAX_VELOCITY_TEST:
            this->testedSamplesCounter++;
            // Moving average.
            this->testedMeanVelocity = this->testedMeanVelocity +
                                       (double)(1 / (double)this->testedSamplesCounter) *
                                           (currentVelocity - this->testedMeanVelocity);

            controlVal = 1.0f; // Force motor into max speed for the test.
            break;
        case DIRECT:
            if (this->maxVelocity != 0)
                controlVal = this->setpoint / this->maxVelocity;
            break;
        case PID_CONTROL:
            controlVal = pid.read(this->setpoint - currentVelocity);
            break;
        }

        // 3. Add some inertia to smooth out wheel movement
        controlVal = fakeInertia.read(controlVal);

        // 4. Write control signal.
        this->motor.write(controlVal);

        // 5. Publish (maybe).
        unsigned int currentEncoderPublishTime = clockNow();

        // Can be false on clock overflow.
        if (currentEncoderPublishTime > previousEncoderPublishTime)
        {
            if (currentEncoderPublishTime - previousEncoderPublishTime > this->encoderPublishRate)
            {
                previousEncoderPublishTime = currentEncoderPublishTime;

                this->rawEncoderMsg.header.seq = seq;
                this->rawEncoderMsg.header.stamp = nh.now();
                this->rawEncoderMsg.acceleration = currentAcceleration;
                this->rawEncoderMsg.velocity = currentVelocity;
                this->rawEncoderMsg.position = currentPosition;
                this->rawEncoderPublisher.publish(&rawEncoderMsg);
                nh.spinOnce();
                seq++;
            }
        }
        else
        {
            // Handle overflow.
            previousEncoderPublishTime = currentEncoderPublishTime;
        }

        // 6. Wait for next update.
        unsigned int currentUpdateTime = clockNow();

        // Can be false on clock overflow.
        if (currentUpdateTime >= previousUpdateTime)
        {
            // How much time passed since start of current update.
            unsigned int diff = currentUpdateTime - previousUpdateTime;
            previousUpdateTime = currentUpdateTime;
            previousPosition = currentPosition;
            previousVelocity = currentVelocity;

            Thread::wait(this->loopUpdateRate - diff); // Loop took "diff" time so wait remaining time.
        }
        else
        {
            // Handle overflow.
            previousUpdateTime = currentUpdateTime;
        }
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

void MotorController::setMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (serviceInProgress)
    {
        res.success = false;
        return;
    }

    serviceInProgress = true;

    if (req.kp >= 0)
        this->pid.setKp(req.kp);

    if (req.ki >= 0)
        this->pid.setKi(req.ki);

    if (req.kd >= 0)
        this->pid.setKd(req.kd);

    if (req.encoder_publish_rate_ms > 0)
        this->encoderPublishRate = req.encoder_publish_rate_ms;

    if (req.loop_update_rate_ms > 0){
        this->loopUpdateRate = req.loop_update_rate_ms;
        pid.setSamplingTime(req.loop_update_rate_ms / 1000.0);
    }
        

    res.success = true;

    serviceInProgress = false;
}