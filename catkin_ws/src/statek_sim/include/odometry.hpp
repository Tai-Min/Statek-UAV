#pragma once

#include <math.h>
#include <chrono>

class Odometry
{
    /**
     * @brief Structure for odometry params.
     */
    struct OdomParams
    {
        float wheelRadius;
        float distanceBetweenWheels;
        unsigned long updateRate;
    };

private:
    bool ready = false;      //!< Whether the loop should start updating.
    bool firstUpdate = true; //!< Whether this is first update of the odometry.

    // Odom stuff.
    float x = 0;      //!< X position.
    float y = 0;      //!< Y position.
    float theta = 0;  //!< Rotation in radians.
    float dx = 0;     //!< Latest change in X.
    float dy = 0;     //!< Latest change in Y.
    float dtheta = 0; //!< Latest rotation change.

    // Parameters.
    float wheelRadius = 0;
    float distanceBetweenWheels = 0;
    unsigned long updateRate = 0;

    // Memory of previous wheel positions in radians.
    float latestLeftWheelPosition = 0;  //!< To compute distance traveled since last update.
    float latestRightWheelPosition = 0; //!< To compute distance traveled since last update.

    // Memory for update loop.
    unsigned long previousUpdateTime; //!< Tracks update time.

    /**
     * @brief Fixes given latest wheel position in case there was unsafe rotation
     * that resulted in encoder's overflow (i.e from 0.1 radians to 6.12 radians).
     * This function changest latest wheel position to accommodate that. 
     * In above example it would return latest position as ~6.38.
     * @param current Newest position.
     * @param latest Latest position
     * @return Latest position with unsafe rotation fixed.
     */
    static float fixLatestWheelPosition(float current, float latest)
    {
        float result = latest;

        // Handle position overflow while moving forward
        // - change previous position a bit so it will be "valid" for acceleration estimation.
        if (result > 5.5 && current < 1)
        {
            result = latest - 2 * M_PI;
        }
        // Handle position overflow while moving backward
        // - change previous position a bit so it will be "valid" for acceleration estimation.
        else if (result < 1 && current > 5.5)
        {
            result = latest + 2 * M_PI;
        }

        return result;
    }

    /**
     * @brief Changes some angular distance to linear distance travelled by the wheel.
     * @param rad Distance travelled by wheel in radians.
     * @return Distance travelled by wheel in meters.
     */
    float angularDistanceToLinear(float rad)
    {
        return rad * this->wheelRadius;
    }

public:
    /**
     * @brief Prepare some stuff for odometry.
     */
    void start()
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        this->previousUpdateTime = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    }

    /**
     * @brief Update odometry.
     */
    void update(double leftPos, double rightPos)
    {
        if (this->firstUpdate)
        {
            this->firstUpdate = false;
            this->latestLeftWheelPosition = leftPos;
            this->latestRightWheelPosition = rightPos;
            return;
        }

        //https://www.hmc.edu/lair/ARW/ARW-Lecture01-Odometry.pdf

        // Get current positions.
        float currentLeftWheelPosition = leftPos;
        float currentRightWheelPosition = rightPos;

        // Fix previous wheel positions if necessary (i.e on overflow from 0 to 2PI).
        this->latestLeftWheelPosition = this->fixLatestWheelPosition(currentLeftWheelPosition, this->latestLeftWheelPosition);
        this->latestRightWheelPosition = this->fixLatestWheelPosition(currentRightWheelPosition, this->latestRightWheelPosition);

        // Get traveled linear distance.
        float leftWheelLinearDistancePassed = this->angularDistanceToLinear(currentLeftWheelPosition - this->latestLeftWheelPosition);
        float rightWheelLinearDistancePassed = this->angularDistanceToLinear(currentRightWheelPosition - this->latestRightWheelPosition);

        float centerLinearDistancePassed = (leftWheelLinearDistancePassed + rightWheelLinearDistancePassed) / 2.0;
        this->dtheta = (rightWheelLinearDistancePassed - leftWheelLinearDistancePassed) / distanceBetweenWheels;

        // Simple Pythagorean angle
        // Assuming that our update time is small enough,
        // we can treat centerLinearDistancePassed like straight line between previous cartesian position
        // and new position. It'll introduce some small error but w/e.
        this->dx = centerLinearDistancePassed * cos(this->theta + this->dtheta / 2.0);
        this->dy = centerLinearDistancePassed * sin(this->theta + this->dtheta / 2.0);

        // Update odom.
        this->x += this->dx;
        this->y += this->dy;
        this->theta += this->dtheta;

        // Save current wheel positions.
        this->latestLeftWheelPosition = currentLeftWheelPosition;
        this->latestRightWheelPosition = currentRightWheelPosition;
    }

    /**
     * @brief Check whether odometry has all required params and is ready to update.
     * @return True if odom ready.
     */
    bool isReady()
    {
        return this->ready;
    }

    /**
     * @brief Set all necessady odometry parameters.
     * 
     * Floats with negative value are ignored.
     * This function sets the ready flag if all of the values are greater than 0.
     * Otherwise it resets this flag.
     * @param params Odometry parameters.
     */
    void setOdomParams(const OdomParams &params)
    {
        this->updateRate = params.updateRate;

        if (params.wheelRadius >= 0)
            this->wheelRadius = params.wheelRadius;

        if (params.distanceBetweenWheels >= 0)
            this->distanceBetweenWheels = params.distanceBetweenWheels;
    };

    /**
     * @brief Get X position from odometry.
     * @return X.
     */
    float getX() const
    {
        return this->x;
    }

    /**
     * @brief Get Y position from odometry.
     * @return Y.
     */
    float getY() const
    {
        return this->y;
    }

    /**
     * @brief Get rotation from odometry.
     * @return Rotation in radians.
     */
    float getTheta() const
    {
        return this->theta;
    }

    /**
     * @brief Get velocity in X direction.
     */
    float getDx() const
    {
        return this->dx / this->updateRate;
    }

    /**
     * @brief Get velocity in Y direction.
     */
    float getDy() const
    {
        return this->dy / this->updateRate;
    }

    /**
     * @brief Get rational velocity.
     */
    float getDtheta() const
    {
        return this->dtheta / this->updateRate;
    }
};