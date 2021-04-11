#include "../include/config.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include "../include/motor_controller.hpp"
#include "../include/odometry.hpp"

#include "../include/ros.hpp"

#include "../lib/ros_lib/statek_msgs/Velocity.h"
#include "../lib/ros_lib/statek_msgs/Encoder.h"
#include "../lib/ros_lib/statek_msgs/RunVelocityTest.h"
#include "../lib/ros_lib/statek_msgs/SetMotorParams.h"
#include "../lib/ros_lib/statek_msgs/RunModelIdentification.h"
#include "../lib/ros_lib/std_srvs/Trigger.h"

#include "../lib/ros_lib/statek_msgs/SetImuParams.h"
#include "../lib/ros_lib/statek_msgs/RunImuCalibration.h"
#include "../lib/ros_lib/sensor_msgs/Imu.h"

#include "../lib/ros_lib/statek_msgs/SetOdomParams.h"
#include "../lib/ros_lib/nav_msgs/Odometry.h"
#include "../lib/ros_lib/tf/transform_broadcaster.h"
#include "../lib/ros_lib/tf/tf.h"

bool imuReady = false; //!< Whether IMU is ready.
unsigned long imuUpdateRate = 0; //!< IMU's sampling time.

/**
 * @brief Arduino's setup function.
 * 
 * Sets all ROS services and prepares all hardware.
 */
void setup();

/**
 * @brief Arduino's loop function.
 */
void loop();

/**
 * @brief Update all the stuff!
 * 
 * Updates ROS, hardware and SAFETY.
 * @param rosPublish Whether allow ROS to publish data. Should be set to false when called from some service.
 */
void update(bool rosPublish = true);

/**
 * @brief Update ROS stuff.
 * @param publish Whether ROS is allowed to publish data.
 */
void updateROS(bool publish = true);

/**
 * @brief Whether hardware stuff is ready, all params loaded etc.
 * @return True if ready.
 */
bool hardwareReady();

/**
 * @brief Try update IMU readings. Can fail if not enough time has passed since last update.
 * @return True if IMU managed to update.
 */
bool tryUpdateImu();

/**
 * @brief Try update all of the hardware.
 * @return True if hardware COULD update. This does not mean that the hardware updated.
 */
bool tryUpdateHardware();

/**
 * @brief Run all safety measurements.
 */
void SAFETY();

// Callbacks for subscribers.
/**
 * @brief Called when ROS serial received new setpoint values from master.
 * @param setpoints Setpoints for both wheels.
 */
void setpointsSubscriberCallback(const statek_msgs::Velocity &setpoints);

// Callbacks for services.
/**
 * @brief Called when user wants to check max velocity. Performs max velocity test for given time.
 * @param req Request structure.
 * @param res Response structure.
 */
void maxVelocityTestServiceCallback(const statek_msgs::RunVelocityTestRequest &req, statek_msgs::RunVelocityTestResponse &res);

/**
 * @brief Called when user wants to identify some motor's dynamics. Collects 100 velocity step response samples for some time.
 * @param mot Motor to perform identification on.
 * @param req Request structure.
 * @param res Response structure.
 */
void stepResponseIdentificationServiceCallback(MotorController &mot, const statek_msgs::RunModelIdentificationRequest &req, statek_msgs::RunModelIdentificationResponse &res);

/**
 * @brief Perform step response identification on left motor.
 * @param req Request structure.
 * @param res Response structure.
 */
void leftMotorStepResponseIdentificationServiceCallback(const statek_msgs::RunModelIdentificationRequest &req, statek_msgs::RunModelIdentificationResponse &res);

/**
 * @brief Perform step response identification on right motor.
 * @param req Request structure.
 * @param res Response structure.
 */
void rightMotorStepResponseIdentificationServiceCallback(const statek_msgs::RunModelIdentificationRequest &req, statek_msgs::RunModelIdentificationResponse &res);

/**
 * @brief Called when user wants to switch to open loop control. Sets open loop control.
 * @param req Request structure.
 * @param res Response structure.
 */
void setDirectControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

/**
 * @brief Called when user wants to switch to closed loop control. Sets closed loop control.
 * @param req Request structure.
 * @param res Response structure.
 */
void setClosedLoopControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

/**
 * @brief Called when user wants to set left motors params. Sets left motor's params.
 * @param req Request structure.
 * @param res Response structure.
 */
void setLeftMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);

/**
 * @brief Called when user wants to set right motors params. Sets right motor's params.
 * @param req Request structure.
 * @param res Response structure.
 */
void setRightMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);

/**
 * @brief Called when user wants to calibrate IMU. Performs IMU calibration.
 * 
 * This function requires that motors have some non zero max velocity!
 * @param req Request structure.
 * @param res Response structure.
 */
void imuCalibrationServiceCallback(const statek_msgs::RunImuCalibrationRequest &req, statek_msgs::RunImuCalibrationResponse &res);

/**
 * @brief Called when user wants to set IMU's parameters. Sets IMU's parameters.
 * @param req Request structure.
 * @param res Response structure.
 */
void setImuParamsServiceCallback(const statek_msgs::SetImuParamsRequest &req, statek_msgs::SetImuParamsResponse &res);

/**
 * @brief Called when user wants to set odom's parameters. Sets odom's parameters.
 * @param req Request structure.
 * @param res Response structure.
 */
void setOdomParamsCallback(const statek_msgs::SetOdomParamsRequest &req, statek_msgs::SetOdomParamsResponse &res);

// Publisher functions
/**
 * @brief Try to publish encoder's state. Can fail if not enough time has passed since last update.
 * @return True on success.
 */
bool tryPublishEncoders();

/**
 * @brief Publish encoder's state to ROS.
 */
void publishEncoders();

/**
 * @brief Try to publish IMU's state. Can fail if not enough time has passed since last update.
 * @return True on success.
 */
bool tryPublishIMU();

/**
 * @brief Publish IMU's state to ROS.
 */
void publishIMU();

/**
 * @brief Try to publish odom's state. Can fail if not enough time has passed since last update.
 * @return True on success.
 */
bool tryPublishOdom();

/**
 * @brief Publish odom's state to ROS along with tf.
 */
void publishOdom();

// Helper functions
/**
 * @brief Force some velocities on both motors and wait for them to accelerate to these velocities.
 * 
 * This functions requires max velocity on both motors to be greater than zero.
 * This function should be called by services not tied to the motors that require some motor movement (i.e mag calibration).
 * @param leftNormalized Value between -1 (max speed reverse) and 1 (max speed forward).
 * @param rightNormalized Value between -1 (max speed reverse) and 1 (max speed forward).
 * @param time How long to wait for motors to accelerate to given speed.
 * @return True on success.
 */
bool forceWheelMovement(float leftNormalized, float rightNormalized, unsigned long time);

// SAFETY functions and variables
bool SAFETY_serviceInProgress = false; //!< Should be set to true during service callback and to false at the end of it. Every service callback should firstly check whether this flag is set to true and if so then fail.
bool SAFETY_stopMotorsFlag = false; //!< Any function related to motor movement should reset this flag as frequent as possible. If not, safety measurements will stop the motors.
int SAFETY_motorsCommunicationFlowSupervisor(); //!< Check whether there is active communication flow between ROS master and this uC. 

// ROS stuff
ros::NodeHandle nh; //!< Manage all ROS stuff.

statek_msgs::Velocity setpoints;
ros::Subscriber<statek_msgs::Velocity>
    setpointsSubscriber(VELOCITY_SETPOINTS_TOPIC, &setpointsSubscriberCallback);

ros::ServiceServer<statek_msgs::RunVelocityTestRequest, statek_msgs::RunVelocityTestResponse>
    maxVelocityTestService(VELOCITY_TEST_SERVICE, &maxVelocityTestServiceCallback);

ros::ServiceServer<statek_msgs::RunModelIdentificationRequest, statek_msgs::RunModelIdentificationResponse>
    leftMotorStepResponseIdentificationService(LEFT_MOTOR_STEP_RESPONSE_IDENTIFICATION_SERVICE, &leftMotorStepResponseIdentificationServiceCallback);

ros::ServiceServer<statek_msgs::RunModelIdentificationRequest, statek_msgs::RunModelIdentificationResponse>
    rightMotorStepResponseIdentificationService(RIGHT_MOTOR_STEP_RESPONSE_IDENTIFICATION_SERVICE, &rightMotorStepResponseIdentificationServiceCallback);

ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
    directControlService(DIRECT_CONTROL_SERVICE, &setDirectControlServiceCallback);

ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
    closedLoopControlService(PID_CONTROL_SERVICE, &setClosedLoopControlServiceCallback);

ros::ServiceServer<statek_msgs::SetMotorParamsRequest, statek_msgs::SetMotorParamsResponse>
    setLeftMotorParamsService(LEFT_MOTOR_PARAM_SERVICE, &setLeftMotorParamsCallback);

ros::ServiceServer<statek_msgs::SetMotorParamsRequest, statek_msgs::SetMotorParamsResponse>
    setrightMotorParamsService(RIGHT_MOTOR_PARAM_SERVICE, &setRightMotorParamsCallback);

ros::ServiceServer<statek_msgs::RunImuCalibrationRequest, statek_msgs::RunImuCalibrationResponse>
    imuCalibrationService(IMU_CALIBRATION_SERVICE, &imuCalibrationServiceCallback);

ros::ServiceServer<statek_msgs::SetImuParamsRequest, statek_msgs::SetImuParamsResponse>
    setImuParamsService(IMU_PARAM_SERVICE, &setImuParamsServiceCallback);

ros::ServiceServer<statek_msgs::SetOdomParamsRequest, statek_msgs::SetOdomParamsResponse>
    setOdomParamsService(ODOM_PARAM_SERVICE, &setOdomParamsCallback);

statek_msgs::Encoder leftEncoderMsg;
ros::Publisher leftEncoderPublisher(LEFT_MOTOR_ENCODER_TOPIC, &leftEncoderMsg);

statek_msgs::Encoder rightEncoderMsg;
ros::Publisher rightEncoderPublisher(RIGHT_MOTOR_ENCODER_TOPIC, &rightEncoderMsg);

sensor_msgs::Imu imuMsg;
ros::Publisher imuPublisher(IMU_TOPIC, &imuMsg);

nav_msgs::Odometry odomMsg;
ros::Publisher odomPublisher(ODOM_TOPIC, &odomMsg);
geometry_msgs::TransformStamped odomTrans;
tf::TransformBroadcaster odomBroadcaster;

MotorController leftMotor(LEFT_MOTOR_GPIO, LEFT_MOTOR_ENCODER_I2C_ADDRESS, true);
MotorController rightMotor(RIGHT_MOTOR_GPIO, RIGHT_MOTOR_ENCODER_I2C_ADDRESS);
MPU9250 imu;
Odometry odom(leftMotor, rightMotor);

void setup()
{
    // Init ROS stuff.
    nh.initNode();
    odomBroadcaster.init(nh);

    nh.subscribe(setpointsSubscriber);
    nh.advertiseService(maxVelocityTestService);
    nh.advertiseService(leftMotorStepResponseIdentificationService);
    nh.advertiseService(rightMotorStepResponseIdentificationService);
    nh.advertiseService(directControlService);
    nh.advertiseService(closedLoopControlService);
    nh.advertiseService(setLeftMotorParamsService);
    nh.advertiseService(setrightMotorParamsService);
    nh.advertise(rightEncoderPublisher);
    nh.advertise(leftEncoderPublisher);

    nh.advertiseService(imuCalibrationService);
    nh.advertiseService(setImuParamsService);
    nh.advertise(imuPublisher);

    nh.advertiseService(setOdomParamsService);
    nh.advertise(odomPublisher);

    // Add some static message values here.
    leftEncoderMsg.header.frame_id = LEFT_MOTOR_TF_LINK;
    rightEncoderMsg.header.frame_id = RIGHT_MOTOR_TF_LINK;

    imuMsg.header.frame_id = IMU_TF_LINK;

    odomMsg.header.frame_id = ODOM_TF_LINK;
    odomMsg.child_frame_id = ODOM_CHILD_LINK;
    odomTrans.header.frame_id = ODOM_TF_LINK;
    odomTrans.child_frame_id = ODOM_CHILD_LINK;

    // Init I2C.
    Wire.begin();

    // Init sensors and actuators.
    leftMotor.start();
    rightMotor.start();
    odom.start();

    imu.setup(IMU_I2C_ADDRESS);
}

void loop()
{
    update();
}

void update(bool rosPublish)
{
    updateROS(rosPublish);
    tryUpdateHardware();
    SAFETY();
}

void updateROS(bool publish)
{
    nh.spinOnce();
    if (publish && hardwareReady())
    {
        tryPublishEncoders();
        tryPublishIMU();
        tryPublishOdom();
    }
}

bool hardwareReady()
{
    bool ready = true;
    ready = ready && leftMotor.isReady() && rightMotor.isReady() && odom.isReady();
    ready = ready && imu.isConnected() && imuReady;
    return ready;
}

bool tryUpdateImu()
{
    static unsigned long previousImuUpdateTime = millis();

    if (!imuReady || imuUpdateRate == 0)
        return false;

    unsigned long now = millis();
    // Handle clock overflow.
    if ((now >= previousImuUpdateTime))
    {
        // Check if 200ms passed so we can update imu.
        if (now - previousImuUpdateTime >= imuUpdateRate)
        {
            imu.update();
            previousImuUpdateTime = now;
            return true;
        }
    }
    else
    {
        previousImuUpdateTime = now;
    }
    return false;
}

bool tryUpdateHardware()
{
    if (!hardwareReady())
        return false;

    leftMotor.tryUpdate();
    rightMotor.tryUpdate();
    tryUpdateImu();
    odom.tryUpdate();

    return true;
}

void SAFETY()
{
    SAFETY_motorsCommunicationFlowSupervisor();
}

// Callbacks for subscribers.
void setpointsSubscriberCallback(const statek_msgs::Velocity &setpoints)
{
    // Don't disturb the service.
    if (SAFETY_serviceInProgress)
    {
        return;
    }

    rightMotor.requestVelocity(setpoints.right);
    leftMotor.requestVelocity(setpoints.left);
    SAFETY_stopMotorsFlag = false; // A message arrived so don's stop the motors.
}

// Callbacks for services.
void maxVelocityTestServiceCallback(const statek_msgs::RunVelocityTestRequest &req, statek_msgs::RunVelocityTestResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }

    SAFETY_serviceInProgress = true;

    MotorController::ControlMode previousMode = rightMotor.getControlMode(); // Restore previous mode after test.

    // Begin the test.
    rightMotor.setControlMode(MotorController::ControlMode::MAX_VELOCITY_TEST);
    leftMotor.setControlMode(MotorController::ControlMode::MAX_VELOCITY_TEST);

    // Let it do it's job for given time.
    unsigned long start = millis();
    unsigned long now = millis();
    bool overflowFlag = false;
    while (now - start < req.time_ms)
    {
        now = millis();
        // Clock overflow
        // Really rare situation.
        // So just mark test as failed.
        if (now < start)
        {
            overflowFlag = true;
            break;
        }

        SAFETY_stopMotorsFlag = false; // Also reset this flag so motor's won't stop due to safety reasons.
        update(false);                 // Keep other parts of the system alive.
    }

    // Restore previous mode.
    rightMotor.setControlMode(previousMode);
    leftMotor.setControlMode(previousMode);

    // Treat mean velocity as maximum velocity to reject noise etc.
    float rightMaxVel = rightMotor.getMaxVelocityTestResult();
    float leftMaxVel = leftMotor.getMaxVelocityTestResult();

    // Set lower value as max possible velocity so both motors will be able to achieve it.
    float maxVel = rightMaxVel;
    if (leftMaxVel < rightMaxVel)
        maxVel = leftMaxVel;

    // Finally, respond to the caller.
    res.success = !overflowFlag;
    res.velocity = maxVel;

    SAFETY_serviceInProgress = false;
}

void stepResponseIdentificationServiceCallback(MotorController &mot, const statek_msgs::RunModelIdentificationRequest &req, statek_msgs::RunModelIdentificationResponse &res)
{
    const uint8_t numSamples = 100;

    // Don't run the test when another callback is doing stuff.
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }

    SAFETY_serviceInProgress = true;

    res.sampling_time = (req.identification_time_ms / 1000.0) / (float)numSamples;
    unsigned int sampleTime = res.sampling_time * 1000;
    bool overflowFlag = false;

    MotorController::ControlMode previousMode = mot.getControlMode();

    // Perform identification.
    mot.requestVelocity(0);
    mot.setControlMode(MotorController::ControlMode::STEP_IDENTIFICATION);

    for (uint8_t i = 0; i < numSamples; i++)
    {
        unsigned long start = millis();
        unsigned long now = start;

        // Wait for next sample.
        while (now - start < sampleTime)
        {
            now = millis();
            // Clock overflow.
            if (now < start)
            {
                overflowFlag = true;
                break;
            }

            SAFETY_stopMotorsFlag = false; // Also reset this flag so motor's won't stop due to safety reasons.
            update(false);                 // Keep other parts of the system alive.
        }

        // Get the sample.
        res.samples[i] = mot.getLatestEncoderState().velocity;
    }

    // Restore previous control mode.
    mot.setControlMode(previousMode);

    res.success = !overflowFlag;

    SAFETY_serviceInProgress = false;
}

void leftMotorStepResponseIdentificationServiceCallback(const statek_msgs::RunModelIdentificationRequest &req, statek_msgs::RunModelIdentificationResponse &res)
{
    stepResponseIdentificationServiceCallback(leftMotor, req, res);
}

void rightMotorStepResponseIdentificationServiceCallback(const statek_msgs::RunModelIdentificationRequest &req, statek_msgs::RunModelIdentificationResponse &res)
{
    stepResponseIdentificationServiceCallback(rightMotor, req, res);
}

void setDirectControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }

    SAFETY_serviceInProgress = true;

    rightMotor.setControlMode(MotorController::ControlMode::DIRECT);
    leftMotor.setControlMode(MotorController::ControlMode::DIRECT);

    res.success = true;

    SAFETY_serviceInProgress = false;
}

void setClosedLoopControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }
    SAFETY_serviceInProgress = true;

    rightMotor.setControlMode(MotorController::ControlMode::PID_CONTROL);
    leftMotor.setControlMode(MotorController::ControlMode::PID_CONTROL);

    res.success = true;

    SAFETY_serviceInProgress = false;
}

void setLeftMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }

    SAFETY_serviceInProgress = true;

    MotorController::ControlParams params = {
        req.kp, req.ki, req.kd, req.smoothing_factor, req.loop_update_rate_ms, req.wheel_max_angular_velocity};

    leftMotor.setMotorParams(params);

    res.success = true;

    SAFETY_serviceInProgress = false;
}

void setRightMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }
    SAFETY_serviceInProgress = true;

    MotorController::ControlParams params = {
        req.kp, req.ki, req.kd, req.smoothing_factor, req.loop_update_rate_ms, req.wheel_max_angular_velocity};

    rightMotor.setMotorParams(params);

    res.success = true;

    SAFETY_serviceInProgress = false;
}

void imuCalibrationServiceCallback(const statek_msgs::RunImuCalibrationRequest &req, statek_msgs::RunImuCalibrationResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }
    SAFETY_serviceInProgress = true;

    // Set motors to forced normalized control.
    // So we can move motors without any parameters set.
    MotorController::ControlMode previousMode = rightMotor.getControlMode();
    leftMotor.setControlMode(MotorController::ControlMode::DIRECT);
    rightMotor.setControlMode(MotorController::ControlMode::DIRECT);

    // Stop the UAV for acc / gyro calibration.
    if (!forceWheelMovement(0, 0, 2000))
    {
        res.success = false;
        SAFETY_serviceInProgress = false;
        return;
    }

    imu.calibrateAccelGyro(); // Calibrate gyro.

    // Move around during mag calibration.
    forceWheelMovement(1, -1, 1500);

    imu.calibrateMag();

    // Stop the motors again.
    forceWheelMovement(0, 0, 2000);
    leftMotor.setControlMode(previousMode);
    rightMotor.setControlMode(previousMode);

    // Save the result!
    res.gyro_bias[0] = imu.getGyroBiasX();
    res.gyro_bias[1] = imu.getGyroBiasY();
    res.gyro_bias[2] = imu.getGyroBiasZ();

    res.acc_bias[0] = imu.getAccBiasX();
    res.acc_bias[1] = imu.getAccBiasY();
    res.acc_bias[2] = imu.getAccBiasZ();

    res.mag_bias[0] = imu.getMagBiasX();
    res.mag_bias[1] = imu.getMagBiasY();
    res.mag_bias[2] = imu.getMagBiasZ();

    res.mag_scale[0] = imu.getMagScaleX();
    res.mag_scale[1] = imu.getMagScaleY();
    res.mag_scale[2] = imu.getMagScaleZ();

    res.success = true;

    SAFETY_serviceInProgress = false;
}

void setImuParamsServiceCallback(const statek_msgs::SetImuParamsRequest &req, statek_msgs::SetImuParamsResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }
    SAFETY_serviceInProgress = true;

    imu.setAccBias(req.acc_bias[0], req.acc_bias[1], req.acc_bias[2]);
    imu.setGyroBias(req.gyro_bias[0], req.gyro_bias[1], req.gyro_bias[2]);
    imu.setMagBias(req.mag_bias[0], req.mag_bias[1], req.mag_bias[2]);
    imu.setMagScale(req.mag_scale[0], req.mag_scale[1], req.mag_scale[2]);
    imu.setMagneticDeclination(req.magnetic_declination);

    imuUpdateRate = req.imu_update_rate_ms;
    if (req.imu_update_rate_ms > 0)
        imuReady = true;
    else
        imuReady = 0;

    res.success = true;

    SAFETY_serviceInProgress = false;
}

void setOdomParamsCallback(const statek_msgs::SetOdomParamsRequest &req, statek_msgs::SetOdomParamsResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (SAFETY_serviceInProgress)
    {
        res.success = false;
        return;
    }
    SAFETY_serviceInProgress = true;

    odom.setOdomParams({req.wheel_radius, req.distance_between_wheels, req.odom_update_rate_ms});

    res.success = true;

    SAFETY_serviceInProgress = false;
}

// Publisher functions.
bool tryPublishEncoders()
{
    static unsigned long previousEncoderPublishTime = millis();

    if (!leftMotor.isReady() || !rightMotor.isReady())
        return false;

    unsigned long now = millis();
    // Handle clock overflow.
    if ((now >= previousEncoderPublishTime) && leftMotor.isReady() && rightMotor.isReady())
    {
        // Check if 100ms passed so we can publish encoders.
        if (now - previousEncoderPublishTime >= 100)
        {
            publishEncoders();
            previousEncoderPublishTime = now;
            return true;
        }
    }
    else
    {
        previousEncoderPublishTime = now;
    }
    return false;
}

void publishEncoders()
{
    static uint32_t seq = 0;

    MotorController::EncoderState leftEncoderState = leftMotor.getLatestEncoderState();
    MotorController::EncoderState rightEncoderState = rightMotor.getLatestEncoderState();

    leftEncoderMsg.header.seq = seq;
    leftEncoderMsg.header.stamp = nh.now();
    leftEncoderMsg.acceleration = leftEncoderState.acceleration;
    leftEncoderMsg.velocity = leftEncoderState.velocity;
    leftEncoderMsg.position = leftEncoderState.position;
    leftEncoderPublisher.publish(&leftEncoderMsg);

    rightEncoderMsg.header.seq = seq;
    rightEncoderMsg.header.stamp = nh.now();
    rightEncoderMsg.acceleration = rightEncoderState.acceleration;
    rightEncoderMsg.velocity = rightEncoderState.velocity;
    rightEncoderMsg.position = rightEncoderState.position;
    rightEncoderPublisher.publish(&rightEncoderMsg);

    seq++;
}

bool tryPublishIMU()
{
    static unsigned long previousImuPublishTime = millis();

    if (!imuReady)
        return false;

    unsigned long now = millis();
    // Handle clock overflow.
    if ((now >= previousImuPublishTime))
    {
        // Check if 100ms passed so we can publish imu data.
        if (now - previousImuPublishTime >= 100)
        {
            publishIMU();
            previousImuPublishTime = now;
            return true;
        }
    }
    else
    {
        previousImuPublishTime = now;
    }
    return false;
}

void publishIMU()
{
    static uint32_t seq = 0;

    imuMsg.header.seq = seq;
    imuMsg.header.stamp = nh.now();

    imuMsg.orientation.w = imu.getQuaternionW();
    imuMsg.orientation.x = imu.getQuaternionX();
    imuMsg.orientation.y = imu.getQuaternionY();
    imuMsg.orientation.z = imu.getQuaternionZ();

    imuMsg.angular_velocity.x = imu.getGyroX() * M_PI / 180;
    imuMsg.angular_velocity.y = imu.getGyroY() * M_PI / 180;
    imuMsg.angular_velocity.z = imu.getGyroZ() * M_PI / 180;

    imuMsg.linear_acceleration.x = imu.getLinearAccX() / 9.81;
    imuMsg.linear_acceleration.y = imu.getLinearAccY() / 9.81;
    imuMsg.linear_acceleration.z = imu.getLinearAccZ() / 9.81;

    imuPublisher.publish(&imuMsg);

    seq++;
}

bool tryPublishOdom()
{
    static unsigned long previousOdomPublishTime = millis();

    if (!odom.isReady())
        return false;

    unsigned long now = millis();
    // Handle clock overflow.
    if ((now >= previousOdomPublishTime))
    {
        // Check if 100ms passed so we can publish odometry.
        if (now - previousOdomPublishTime >= 100)
        {
            publishOdom();
            previousOdomPublishTime = now;
            return true;
        }
    }
    else
    {
        previousOdomPublishTime = now;
    }
    return false;
}

void publishOdom()
{
    static uint32_t seq = 0;

    // Odometry message
    odomMsg.header.seq = seq;
    odomMsg.header.stamp = nh.now();

    odomMsg.pose.pose.position.x = odom.getX();
    odomMsg.pose.pose.position.y = odom.getY();
    odomMsg.pose.pose.position.z = 0;
    odomMsg.pose.pose.orientation = tf::createQuaternionFromYaw(odom.getTheta());

    odomMsg.twist.twist.linear.x = odom.getDx();
    odomMsg.twist.twist.linear.y = odom.getDy();
    odomMsg.twist.twist.angular.z = odom.getDtheta();

    odomPublisher.publish(&odomMsg);

    // Tf
    odomTrans.header.seq = seq;
    odomTrans.header.stamp = nh.now();

    odomTrans.transform.translation.x = odom.getX();
    odomTrans.transform.translation.y = odom.getY();
    odomTrans.transform.translation.z = 0;
    odomTrans.transform.rotation = tf::createQuaternionFromYaw(odom.getTheta());

    odomBroadcaster.sendTransform(odomTrans);

    seq++;
}

// Helpers
bool forceWheelMovement(float leftNormalized, float rightNormalized, unsigned long time)
{
    if ((leftMotor.getMaxVelocity() == 0 && leftNormalized != 0) || (rightMotor.getMaxVelocity() == 0 && rightNormalized != 0))
    {
        nh.logerror("Can't force movement of the wheels (wheel_max_angular_velocity not set).");
        return false;
    }

    leftMotor.requestVelocity(leftNormalized * leftMotor.getMaxVelocity());
    rightMotor.requestVelocity(rightNormalized * rightMotor.getMaxVelocity());

    // Wait for wheels a bit to start moving.
    unsigned long start = millis();
    unsigned long now = millis();
    while (now - start < time)
    {
        now = millis();

        // Clock overflow.
        // Just wait longer.
        if (now < start)
        {
            start = millis();
            now = start;
            break;
        }

        SAFETY_stopMotorsFlag = false; // Also reset this flag so motor's won't stop due to safety reasons.
        update(false);                 // Keep other parts of the system alive.
    }
    return true;
}

// SAFETY functions.
int SAFETY_motorsCommunicationFlowSupervisor()
{
    static unsigned long previousCommunicationCheckTime = millis();

    unsigned long now = millis();

    // Handle clock overflow.
    if ((now >= previousCommunicationCheckTime))
    {
        // Check if 500ms has passed so we can check safety function.
        if (now - previousCommunicationCheckTime >= 500)
        {
            // The flag was not reset by motors/vel_cmd subscriber
            // so treat it as broken communication flow and stop the motors.
            if (SAFETY_stopMotorsFlag)
            {
                // Display message only if there were non zero velocities set to motors.
                if (leftMotor.getRequestedVelocity() != 0 || rightMotor.getRequestedVelocity() != 0)
                    nh.logwarn("Communication flow broken.");

                rightMotor.requestVelocity(0);
                leftMotor.requestVelocity(0);

                previousCommunicationCheckTime = now;
                return -1; // Communication broken.
            }
            // The flag was reset by motors/vel_cmd callback
            // so communication flow is ok.
            else
            {
                SAFETY_stopMotorsFlag = true; // Set it again and wait for motors/vel_cmd to reset it hopefully.
                previousCommunicationCheckTime = now;
                return 1; // Communication active.
            }
        }
    }
    else
    {
        previousCommunicationCheckTime = now;
    }
    return 0; // Didn't check the flag.
}