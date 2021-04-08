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

bool imuReady = false;
unsigned long imuUpdateRate = 0;

void setup();
void loop();
void update(bool rosPublish = true);
void updateROS(bool publish = true);
bool hardwareReady();
bool tryUpdateImu();
bool tryUpdateHardware();
void SAFETY();

// Callbacks for subscribers.
void setpointsSubscriberCallback(const statek_msgs::Velocity &setpoints);

// Callbacks for services.
void maxVelocityTestServiceCallback(const statek_msgs::RunVelocityTestRequest &req, statek_msgs::RunVelocityTestResponse &res);
void stepResponseIdentificationServiceCallback(const statek_msgs::RunModelIdentificationRequest &req, statek_msgs::RunModelIdentificationResponse &res);
void setDirectControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
void setClosedLoopControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
void setLeftMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);
void setRightMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);

void imuCalibrationServiceCallback(const statek_msgs::RunImuCalibrationRequest &req, statek_msgs::RunImuCalibrationResponse &res);
void setImuParamsServiceCallback(const statek_msgs::SetImuParamsRequest &req, statek_msgs::SetImuParamsResponse &res);

void setOdomParamsCallback(const statek_msgs::SetOdomParamsRequest &req, statek_msgs::SetOdomParamsResponse &res);

// Publisher functions
bool tryPublishEncoders();
void publishEncoders();

bool tryPublishIMU();
void publishIMU();

bool tryPublishOdom();
void publishOdom();

// Helper functions
bool forceWheelMovement(float left, float right, unsigned long time);

// SAFETY functions and variables
bool SAFETY_serviceInProgress = false;
bool SAFETY_stopMotorsFlag = false;
int SAFETY_motorsCommunicationFlowSupervisor();

// ROS stuff
ros::NodeHandle nh;

statek_msgs::Velocity setpoints;
ros::Subscriber<statek_msgs::Velocity>
    setpointsSubscriber(VELOCITY_SETPOINTS_TOPIC, &setpointsSubscriberCallback);

ros::ServiceServer<statek_msgs::RunVelocityTestRequest, statek_msgs::RunVelocityTestResponse>
    maxVelocityTestService(VELOCITY_TEST_SERVICE, &maxVelocityTestServiceCallback);

ros::ServiceServer<statek_msgs::RunModelIdentificationRequest, statek_msgs::RunModelIdentificationResponse>
    stepResponseIdentificationService(STEP_RESPONSE_IDENTIFICATION_SERVICE, &stepResponseIdentificationServiceCallback);

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
Odometry odom(leftMotor, rightMotor, imu);

void setup()
{
    // Init ROS stuff.
    nh.initNode();

    nh.subscribe(setpointsSubscriber);
    nh.advertiseService(maxVelocityTestService);
    nh.advertiseService(stepResponseIdentificationService);
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
        //tryPublishOdom();
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
    //odom.tryUpdate();

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

void stepResponseIdentificationServiceCallback(const statek_msgs::RunModelIdentificationRequest &req, statek_msgs::RunModelIdentificationResponse &res)
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

    MotorController::ControlMode previousMode = rightMotor.getControlMode();

    if (req.selected_motor == statek_msgs::RunModelIdentificationRequest::MOTOR_LEFT)
    {
        // Stop right motor.
        rightMotor.setControlMode(MotorController::ControlMode::DIRECT);
        rightMotor.requestVelocity(0);

        // Perform identification.
        leftMotor.requestVelocity(0);
        leftMotor.setControlMode(MotorController::ControlMode::STEP_IDENTIFICATION);
    }
    else
    {
        // Stop left motor.
        leftMotor.setControlMode(MotorController::ControlMode::DIRECT);
        leftMotor.requestVelocity(0);

        // Perform identification.
        rightMotor.requestVelocity(0);
        rightMotor.setControlMode(MotorController::ControlMode::STEP_IDENTIFICATION);
    }

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
        if (req.selected_motor == statek_msgs::RunModelIdentificationRequest::MOTOR_LEFT)
        {
            res.samples[i] = leftMotor.getLatestEncoderState().velocity;
        }
        else
        {
            res.samples[i] = rightMotor.getLatestEncoderState().velocity;
        }
    }

    // Restore previous control mode.
    leftMotor.setControlMode(previousMode);
    rightMotor.setControlMode(previousMode);

    res.success = !overflowFlag;

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

    // Set motors to direct control.
    MotorController::ControlMode previousMode = rightMotor.getControlMode();
    leftMotor.setControlMode(MotorController::ControlMode::DIRECT);
    rightMotor.setControlMode(MotorController::ControlMode::DIRECT);

    // Stop the UAV for acc / gyro calibration.
    if(!forceWheelMovement(0, 0, 2000)){
        res.success = false;
        SAFETY_serviceInProgress = false;
        return;
    }

    imu.calibrateAccelGyro(); // Calibrate gyro.

    // Move around during mag calibration.
    forceWheelMovement(leftMotor.getMaxVelocity(), -rightMotor.getMaxVelocity(), 1500);

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
    imuReady = true;

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

bool tryPublishOdom(){
    static unsigned long previousOdomPublishTime = millis();

    unsigned long now = millis();
    // Handle clock overflow.
    if ((now >= previousOdomPublishTime))
    {
        // Check if 50ms passed so we can publish odometry.
        if (now - previousOdomPublishTime >= 50)
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

void publishOdom(){
    static uint32_t seq = 0;

    // Odometry message
    odomMsg.header.seq = seq;
    odomMsg.header.stamp = nh.now();

    odomMsg.pose.pose.position.x = odom.getX();
    odomMsg.pose.pose.position.y = odom.getY();
    odomMsg.pose.pose.position.z = 0;
    odomMsg.pose.pose.orientation.w = imu.getQuaternionW();
    odomMsg.pose.pose.orientation.x = imu.getQuaternionX();
    odomMsg.pose.pose.orientation.y = imu.getQuaternionY();
    odomMsg.pose.pose.orientation.z = imu.getQuaternionZ();

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
    odomTrans.transform.rotation.w = imu.getQuaternionW();
    odomTrans.transform.rotation.x = imu.getQuaternionX();
    odomTrans.transform.rotation.y = imu.getQuaternionY();
    odomTrans.transform.rotation.z = imu.getQuaternionZ();

    seq++;
}

// Helpers
bool forceWheelMovement(float left, float right, unsigned long time)
{
    if ((leftMotor.getMaxVelocity() == 0 && left != 0) || (rightMotor.getMaxVelocity() == 0 && right != 0))
    {
        nh.logerror("Can't force movement of the wheels (Max velocity not set).");
        return false;
    }

    // Rotate around for mag calibration.
    leftMotor.requestVelocity(left);
    rightMotor.requestVelocity(right);

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