#pragma once

#include "motor_controller.hpp"
#include <MPU9250.h>

class Odometry
{
    struct OdomParams
    {
        float wheelRadius;
        float distanceBetweenWheels;
        unsigned long updateRate;
    };

private:
    bool ready = false;

    // Odom stuff.
    float x = 0, y = 0, theta = 0;
    float latestDx = 0, latestDy = 0, latestDtheta = 0;

    // Parameters.
    float wheelRadius = 0;
    float distanceBetweenWheels = 0;
    unsigned long updateRate;

    // Memory of previous wheel positions in radians.
    float latestLeftWheelPosition = 0;
    float latestRightWheelPosition = 0;

    // Memory for update loop.
    unsigned long previousUpdateTime;

    // Motors for odometry.
    MotorController &leftMotor;
    MotorController &rightMotor;
    MPU9250 &imu;

    void update()
    {
        //https://www.hmc.edu/lair/ARW/ARW-Lecture01-Odometry.pdf

        // Get current positions.
        float currentLeftWheelPosition = leftMotor.getLatestEncoderState().position;
        float currentRightWheelPosition = rightMotor.getLatestEncoderState().position;

        // Fix previous wheel positions if necessary (i.e on overflow from 0 to 2PI).
        this->latestLeftWheelPosition = this->fixLatestWheelPosition(currentLeftWheelPosition, this->latestLeftWheelPosition);
        this->latestRightWheelPosition = this->fixLatestWheelPosition(currentRightWheelPosition, this->latestRightWheelPosition);

        // Get traveled linear distance.
        float leftWheelLinearDistancePassed = this->angularDistanceToLinear(currentLeftWheelPosition - this->latestLeftWheelPosition);
        float rightWheelLinearDistancePassed = this->angularDistanceToLinear(currentRightWheelPosition - this->latestLeftWheelPosition);

        float centerLinearDistancePassed = (leftWheelLinearDistancePassed + rightWheelLinearDistancePassed) / 2.0;
        this->latestDtheta = (leftWheelLinearDistancePassed + rightWheelLinearDistancePassed) / distanceBetweenWheels;

        // Simple Pythagorean angle
        // Assuming that our update time is small enough,
        // we can treat centerLinearDistancePassed like straight line between previous cartesian position
        // and new position. It'll introduce some small error but w/e.
        this->latestDx = centerLinearDistancePassed * cos(this->theta + this->latestDtheta / 2.0);
        this->latestDy = centerLinearDistancePassed * sin(this->theta + this->latestDtheta / 2.0);

        // Update odom.
        this->x += this->latestDx;
        this->y += this->latestDy;
        this->theta = imu.getYaw() * M_PI / 180; // Face true north and convert to radians.

        // Save current wheel positions.
        this->latestLeftWheelPosition = currentLeftWheelPosition;
        this->latestRightWheelPosition = currentRightWheelPosition;  
    }

    static float fixLatestWheelPosition(float current, float latest)
    {
        float result = latest;

        // Handle position overflow while moving forward
        // - change previous position a bit so it will be "valid" for acceleration estimation.
        if (result > 5.5 && current < 1)
        {
            result = current - 2 * M_PI;
        }
        // Handle position overflow while moving backward
        // - change previous position a bit so it will be "valid" for acceleration estimation.
        else if (result < 1 && current > 5.5)
        {
            result = current + 2 * M_PI;
        }

        return result;
    }

    float angularDistanceToLinear(float rad)
    {
        return rad * this->wheelRadius;
    }

public:
    Odometry(MotorController &_leftMotor, MotorController &_rightMotor, MPU9250 &_imu)
        : leftMotor(_leftMotor), rightMotor(_rightMotor), imu(_imu) {}

    void start()
    {
        this->previousUpdateTime = millis();
    }

    bool tryUpdate()
    {
        if (!this->ready || this->updateRate == 0)
            return false;

        unsigned long now = millis();

        // Clock overflow check.
        if (now >= this->previousUpdateTime)
        {
            if (now - this->previousUpdateTime >= this->updateRate)
            {
                this->update();
                this->previousUpdateTime = now;
                return true;
            }
        }
        else
        {
            this->previousUpdateTime = now;
        }
        return false;
    }

    bool isReady()
    {
        return this->ready;
    }

    void setOdomParams(const OdomParams &params)
    {
        if (params.updateRate > 0)
            this->updateRate = params.updateRate;
        if (params.wheelRadius > 0)
            this->wheelRadius = params.wheelRadius;
        if (params.distanceBetweenWheels > 0)
            this->distanceBetweenWheels = params.distanceBetweenWheels;

        this->ready = true;
    };

    float getX()
    {
        return this->x;
    }

    float getY()
    {
        return this->y;
    }

    float getTheta()
    {
        return this->theta;
    }

    float getDx(){
        return this->latestDx;
    }

    float getDy(){
        return this->latestDy;
    }

    float getDtheta(){
        return this->latestDtheta;
    }
};