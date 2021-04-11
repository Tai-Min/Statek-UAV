#include "../include/motor_controller.hpp"

MotorController::MotorController(const Motor::Gpio &motorGpio, uint8_t encoderAddr, bool _reverseEncoder, TwoWire &encoderI2c)
    : motor(motorGpio), encoder(encoderAddr, encoderI2c), reverseEncoder(_reverseEncoder),
      fakeInertia(0.18), pid(-1, 1) {}

MotorController::EncoderState MotorController::getCurrentEncoderState(bool &ok) const
{
    EncoderState currentEncoderState;

    currentEncoderState.position = this->encoder.absolutePosition(ok);
    if (!ok)
    {
        ok = false;
        return currentEncoderState;
    }

    // Inverse encoder's output if necessary.
    if (this->reverseEncoder)
        currentEncoderState.position = 2 * M_PI - currentEncoderState.position;

    float fixedLatestPosition = this->latestEncoderState.position;

    // Handle position overflow while moving forward
    // - change previous position a bit so it will be "valid" for acceleration estimation.
    if (fixedLatestPosition > 5.5 && currentEncoderState.position < 1)
    {
        fixedLatestPosition = this->latestEncoderState.position - 2 * M_PI;
    }
    // Handle position overflow while moving backward
    // - change previous position a bit so it will be "valid" for acceleration estimation.
    else if (fixedLatestPosition < 1 && currentEncoderState.position > 5.5)
    {
        fixedLatestPosition = this->latestEncoderState.position + 2 * M_PI;
    }

    currentEncoderState.velocity = this->encoder.velocity(ok);
    if (!ok)
    {
        ok = false;
        return currentEncoderState;
    }

    // Find the sign of the velocity
    if (currentEncoderState.position < fixedLatestPosition)
    {
        currentEncoderState.velocity *= -1;
    }

    // Maybe add Kalman filter later?
    //float accelerationFromPosition = currentEncoderState.position - fixedLatestPosition - this->latestEncoderState.velocity * this->loopUpdateRate / (float)1000.0;
    float accelerationFromVelocity = (currentEncoderState.velocity - this->latestEncoderState.velocity) * this->loopUpdateRate / (float)1000.0;
    //currentEncoderState.acceleration = (accelerationFromPosition + accelerationFromVelocity) / 2.0;
    currentEncoderState.acceleration = accelerationFromVelocity;

    ok = true;
    return currentEncoderState;
}

float MotorController::getControlValue(const EncoderState &currentEncoderState)
{
    float controlVal = 0;
    switch (this->controlMode)
    {
    case MAX_VELOCITY_TEST:
        this->movingAverageSampleTracker++;

        // Moving average.
        this->averageVelocity = this->averageVelocity +
                                (float)(1 / (float)this->movingAverageSampleTracker) *
                                    (currentEncoderState.velocity - this->averageVelocity);

        controlVal = 1.0f; // Force motor into max speed during this test.
        break;
    case STEP_IDENTIFICATION:
        controlVal = 1;
        break;
    case DIRECT:
        // Simply scale the setpoint to range [-1; 1]
        if (this->maxVelocity != 0)
            controlVal = this->setpoint / this->maxVelocity;
        break;
    case PID_CONTROL:
        controlVal = this->pid.read(this->setpoint - currentEncoderState.velocity);
        break;
    }

    // Smooth out the movement.
    // So the chassis won't break by sudden energy from motors :).
    controlVal = this->fakeInertia.read(controlVal);

    // Just in case.
    controlVal = this->saturate(-1, 1, controlVal);
    return controlVal;
}

bool MotorController::update()
{
    // 1. Read / estimate system's state.
    bool ok = true;
    EncoderState currentEncoderState = this->getCurrentEncoderState(ok);
    if (!ok)
    {
        this->setpoint = 0;
    }

    // 2. Control.
    float controlVal = this->getControlValue(currentEncoderState);
    motor.write(controlVal);

    // 3. Keep current motion dynamics for next iteration
    // and for anyone interested.
    if (ok)
    {
        this->latestEncoderState.position = currentEncoderState.position;
        this->latestEncoderState.velocity = currentEncoderState.velocity;
        this->latestEncoderState.acceleration = currentEncoderState.acceleration;
    }
    else
    {
        return false; // Encoder reading failed.
    }
    return true;
}

float MotorController::saturate(float minVal, float maxVal, float val)
{
    if (val > maxVal)
        val = maxVal;
    if (val < minVal)
        val = minVal;
    return val;
}

void MotorController::start()
{
    this->previousUpdateTime = millis();
    this->motor.enable();
}

bool MotorController::isReady() const
{
    return this->ready;
}

MotorController::FailCode MotorController::tryUpdate()
{
    // Something went wrong / controller has not received any params.
    if (!this->ready)
        return MOTOR_NOT_READY;

    unsigned long now = millis();
    // Handle clock overflow.
    if (now >= this->previousUpdateTime)
    {
        // Check if enough time has passed so we can update control loop.
        if (now - this->previousUpdateTime > this->loopUpdateRate)
        {
            if (this->update())
                return CONTROL_UPDATED;
            else
                return ENCODER_FAILURE;

            this->previousUpdateTime = now;
        }
    }
    else
    {
        this->previousUpdateTime = now;
    }
    return UPDATE_NOT_READY;
}

void MotorController::requestVelocity(float vel)
{
    this->setpoint = this->saturate(-1 * this->maxVelocity, this->maxVelocity, vel);
}

void MotorController::setMotorParams(const ControlParams &params)
{
    if (params.kp >= 0)
        this->pid.setKp(params.kp);

    if (params.ki >= 0)
        this->pid.setKi(params.ki);

    if (params.kd >= 0)
        this->pid.setKd(params.kd);

    if (params.loopUpdateRate > 0)
    {
        this->loopUpdateRate = params.loopUpdateRate;
        this->pid.setSamplingTime(params.loopUpdateRate / (float)1000.0);
    }

    if (params.smoothingFactor >= 0)
        this->fakeInertia.setSmoothingFactor(params.smoothingFactor);

    if (params.maxVelocity >= 0)
        this->maxVelocity = params.maxVelocity;

    if (this->maxVelocity != 0 && this->loopUpdateRate != 0)
        this->ready = true;
    else
        this->ready = false;
}

void MotorController::setControlMode(ControlMode cm)
{
    // Reset moving average for velocity test
    if (cm == ControlMode::MAX_VELOCITY_TEST)
    {
        this->averageVelocity = 0;
        this->movingAverageSampleTracker = 0;
    }

    // Reset setpoint for safety.
    this->setpoint = 0;

    this->controlMode = cm;
}

float MotorController::getRequestedVelocity() const
{
    return this->setpoint;
}

MotorController::ControlMode MotorController::getControlMode() const
{
    return this->controlMode;
}

MotorController::EncoderState MotorController::getLatestEncoderState() const
{
    return this->latestEncoderState;
}

float MotorController::getMaxVelocity() const
{
    return this->maxVelocity;
}

float MotorController::getMaxVelocityTestResult() const
{
    return averageVelocity;
}