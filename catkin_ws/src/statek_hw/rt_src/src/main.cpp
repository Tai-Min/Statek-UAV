#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>

#include "../include/config.hpp"
#include "../include/motor_controller.hpp"

#include "../lib/ros_lib/ros.h"
#include "../lib/ros_lib/statek_msgs/Velocity.h"
#include "../lib/ros_lib/statek_msgs/Encoder.h"
#include "../lib/ros_lib/statek_msgs/RunVelocityTest.h"
#include "../lib/ros_lib/statek_msgs/SetMotorParams.h"
#include "../lib/ros_lib/statek_msgs/SetImuParams.h"
#include "../lib/ros_lib/statek_msgs/RunImuCalibration.h"
#include "../lib/ros_lib/std_srvs/Trigger.h"
#include "../lib/ros_lib/sensor_msgs/Imu.h"

void setup();
void loop();
void delayAndSpin(unsigned long t);

// Param loaders.
bool loadParamsToMotorControllers();

// Callbacks for subscribers.
void setpointsSubscriberCallback(const statek_msgs::Velocity &setpoints);

// Callbacks for services.
void maxVelocityTestServiceCallback(const statek_msgs::RunVelocityTestRequest &req, statek_msgs::RunVelocityTestResponse &res);
void setDirectControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
void setClosedLoopControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
void setLeftMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);
void setRightMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);
void imuCalibrationServiceCallback(const statek_msgs::RunImuCalibrationRequest &req, statek_msgs::RunImuCalibrationResponse &res);
void setImuParamsServiceCallback(const statek_msgs::SetImuParamsRequest &req, statek_msgs::SetImuParamsResponse &res);

// Publisher functions
bool tryPublishEncoders();
void publishEncoders();
bool tryPublishIMU();
void publishIMU();

// SAFETY functions and variables
bool SAFETY_serviceInProgress = false;
bool SAFETY_stopMotorsFlag = false;
int SAFETY_communicationFlowSupervisor();
int SAFETY_tryForceI2cRecovery();
bool SAFETY_forceI2cRecovery();

ros::NodeHandle nh;

statek_msgs::Velocity setpoints;
ros::Subscriber<statek_msgs::Velocity>
    setpointsSubscriber(VELOCITY_SETPOINTS_TOPIC, &setpointsSubscriberCallback);

ros::ServiceServer<statek_msgs::RunVelocityTestRequest, statek_msgs::RunVelocityTestResponse>
    maxVelocityTestService(VELOCITY_TEST_SERVICE, &maxVelocityTestServiceCallback);

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

statek_msgs::Encoder leftEncoderMsg;
ros::Publisher leftEncoderPublisher(LEFT_MOTOR_ENCODER_TOPIC, &leftEncoderMsg);

statek_msgs::Encoder rightEncoderMsg;
ros::Publisher rightEncoderPublisher(RIGHT_MOTOR_ENCODER_TOPIC, &rightEncoderMsg);

sensor_msgs::Imu imuMsg;
ros::Publisher imuPublisher(IMU_TOPIC, &imuMsg);

MotorController leftMotor(LEFT_MOTOR_GPIO, LEFT_MOTOR_ENCODER_I2C_ADDRESS, true);
MotorController rightMotor(RIGHT_MOTOR_GPIO, RIGHT_MOTOR_ENCODER_I2C_ADDRESS);
MPU9250 imu;

AM4096 encoder(LEFT_MOTOR_ENCODER_I2C_ADDRESS);

void setup()
{
    leftMotor.start();
    rightMotor.start();

    imu.setup(IMU_I2C_ADDRESS);

    // Init I2C.
    Wire.begin();

    // Init ROS stuff.
    nh.initNode();

    nh.subscribe(setpointsSubscriber);
    nh.advertiseService(maxVelocityTestService);
    nh.advertiseService(directControlService);
    nh.advertiseService(closedLoopControlService);
    nh.advertiseService(setLeftMotorParamsService);
    nh.advertiseService(setrightMotorParamsService);
    nh.advertise(rightEncoderPublisher);
    nh.advertise(leftEncoderPublisher);
    nh.advertise(imuPublisher);

    // Add some static message values here.
    leftEncoderMsg.header.frame_id = LEFT_MOTOR_TF_LINK;
    rightEncoderMsg.header.frame_id = RIGHT_MOTOR_TF_LINK;
    imuMsg.header.frame_id = IMU_TF_LINK;

    Serial.begin(9600);
}

void loop()
{
    bool ok;
    Serial.println(encoder.absolutePosition(ok));
    // ROS update.
    tryPublishEncoders();
    tryPublishIMU();

    // Sensors / actuators update.
    MotorController::FailCode code = leftMotor.tryUpdate();
    if (code == MotorController::FailCode::ENCODER_FAILURE)
        goto recover;

    code = rightMotor.tryUpdate();
    if (code == MotorController::FailCode::ENCODER_FAILURE)
        goto recover;

    if (!imu.update() && digitalRead(D_SDA) == LOW)
        goto recover;

    // Safety measurements.
    SAFETY_communicationFlowSupervisor();
    SAFETY_tryForceI2cRecovery();

    return;

recover:
    SAFETY_forceI2cRecovery();
    return;
}

void delayAndSpin(unsigned long t)
{
    for (unsigned long i = 0; i < t; i++)
    {
        nh.spinOnce();
        delay(1);
    }
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

    // Let it do it's job for some time.

    for (unsigned int i = 0; i < req.time; i += 10)
    {
        SAFETY_stopMotorsFlag = false; // Also reset this flag so motor's won't stop due to safety reasons.
        loop();                        // Keep other parts of the system alive.
        delay(9);                      // One ms delay is in loop.
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
    res.success = true;
    res.velocity = maxVel;

    SAFETY_serviceInProgress = false;
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

    nh.logwarn("Params received!");

    MotorController::ControlParams params = {
        req.kp, req.ki, req.kd, req.loop_update_rate_ms, req.wheel_max_angular_velocity};

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
        req.kp, req.ki, req.kd, req.loop_update_rate_ms, req.wheel_max_angular_velocity};

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

    // Stop the UAV for acc / gyro calibration.
    leftMotor.requestVelocity(0);
    rightMotor.requestVelocity(0);

    imu.calibrateAccelGyro();

    // Rotate around for mag calibration.
    leftMotor.requestVelocity(-1000);
    rightMotor.requestVelocity(1000);

    imu.calibrateMag();

    // Stop again.
    leftMotor.requestVelocity(0);
    rightMotor.requestVelocity(0);

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

    res.success = true;

    SAFETY_serviceInProgress = false;
}

// Publisher functions
bool tryPublishEncoders()
{
    static unsigned long long previousEncoderPublishTime = millis();

    unsigned long long now = millis();
    // Handle clock overflow.
    if ((now >= previousEncoderPublishTime))
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
    static unsigned long long previousImuPublishTime = millis();

    unsigned long long now = millis();
    // Handle clock overflow.
    if ((now >= previousImuPublishTime))
    {
        // Check if 100ms passed so we can publish encoders.
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

    imuMsg.angular_velocity.x = imu.getGyroX();
    imuMsg.angular_velocity.y = imu.getGyroY();
    imuMsg.angular_velocity.z = imu.getGyroZ();

    imuMsg.linear_acceleration.x = imu.getLinearAccX();
    imuMsg.linear_acceleration.y = imu.getLinearAccY();
    imuMsg.linear_acceleration.z = imu.getLinearAccZ();

    imuPublisher.publish(&imuMsg);

    seq++;
}

int SAFETY_communicationFlowSupervisor()
{
    static unsigned long long previousCommunicationCheckTime = millis();

    unsigned long long now = millis();

    // Handle clock overflow.
    if ((now >= previousCommunicationCheckTime))
    {
        // Check if 100ms passed so we can check safety function.
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
                return -1; // Communication broken.
            }
            // The flag was reset by motors/vel_cmd callback
            // so communication flow is ok.
            else
            {
                SAFETY_stopMotorsFlag = true; // Set it again and wait for motors/vel_cmd to reset it hopefully.
                return 1;                     // Communication active.
            }
            previousCommunicationCheckTime = now;
        }
    }
    else
    {
        previousCommunicationCheckTime = now;
    }
    return 0; // Didn't check the flag.
}

int SAFETY_tryForceI2cRecovery()
{
    static unsigned long long previousCommunicationCheckTime = millis();

    unsigned long long now = millis();

    // Handle clock overflow.
    if ((now >= previousCommunicationCheckTime))
    {
        // Check if 100ms passed so we can force reset i2c.
        if (now - previousCommunicationCheckTime >= 100)
        {
            previousCommunicationCheckTime = now;
            if (SAFETY_forceI2cRecovery())
                return 1; // Recovery success.
            else
                return -1; // Recovery failed.
        }
    }
    else
    {
        previousCommunicationCheckTime = now;
    }
    return 0; // Didn't try to recovery.
}

bool SAFETY_forceI2cRecovery()
{
    pinMode(D_SDA, OUTPUT);
    pinMode(D_SCL, OUTPUT);

    // Perform max 9 nine clock pulses
    // to unstuck the line.
    for (uint8_t i = 0; i < 9; i++)
    {
        digitalWrite(D_SDA, HIGH);
        delayMicroseconds(50);

        // SDA is high so success!
        if (digitalRead(D_SDA) == HIGH)
        {
            return true;
        }
        // SDA still low so generate pulse.
        else
        {
            digitalWrite(D_SCL, HIGH);
            delayMicroseconds(50);
            digitalWrite(D_SCL, LOW);
            delayMicroseconds(50);
            digitalWrite(D_SCL, HIGH);
            delayMicroseconds(50);
        }
    }

    // Restore I2C line and generate STOP condition.
    Wire.endTransmission();

    if (digitalRead(D_SDA) == LOW)
        return false;
    return true;
}