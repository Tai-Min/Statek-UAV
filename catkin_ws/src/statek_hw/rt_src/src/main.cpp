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
#include "../lib/ros_lib/std_srvs/Trigger.h"
#include "../lib/ros_lib/sensor_msgs/Imu.h"

void setup();
void loop();

// Param loaders.
void loadParamsToMotorControllers();

// Callbacks for subscribers.
void setpointsSubscriberCallback(const statek_msgs::Velocity &setpoints);

// Callbacks for services.
void maxVelocityTestServiceCallback(const statek_msgs::RunVelocityTestRequest &req, statek_msgs::RunVelocityTestResponse &res);
void setDirectControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
void setClosedLoopControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
void setLeftMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);
void setRightMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);

// Publisher functions
bool tryPublishEncoders();
void publishEncoders();
bool tryPublishIMU();
void publishIMU();

// SAFETY functions and variables
bool SAFETY_serviceInProgress = false;
bool SAFETY_stopMotorsFlag = false;
void SAFETY_communicationFlowSupervisor();
void SAFETY_recoverI2C();

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

statek_msgs::Encoder leftEncoderMsg;
ros::Publisher leftEncoderPublisher(LEFT_MOTOR_ENCODER_TOPIC, &leftEncoderMsg);

statek_msgs::Encoder rightEncoderMsg;
ros::Publisher rightEncoderPublisher(RIGHT_MOTOR_ENCODER_TOPIC, &rightEncoderMsg);

sensor_msgs::Imu imuMsg;
ros::Publisher imuPublisher(IMU_TOPIC, &imuMsg);

MotorController leftMotor(LEFT_MOTOR_GPIO, Wire, LEFT_MOTOR_ENCODER_I2C_ADDRESS, true);
MotorController rightMotor(LEFT_MOTOR_GPIO, Wire, LEFT_MOTOR_ENCODER_I2C_ADDRESS, true);
MPU9250 imu;

void setup()
{
    // Init I2C.
    Wire.begin();

    // Init ROS stuff.
    nh.initNode();
    /*while (!nh.connected())
    {
    }*/

    nh.subscribe(setpointsSubscriber);
    nh.advertiseService(maxVelocityTestService);
    nh.advertiseService(directControlService);
    nh.advertiseService(closedLoopControlService);
    nh.advertiseService(setLeftMotorParamsService);
    nh.advertiseService(setrightMotorParamsService);
    nh.advertise(rightEncoderPublisher);
    nh.advertise(leftEncoderPublisher);
    nh.advertise(imuPublisher);

    loadParamsToMotorControllers();

    // Add some static message values here.
    leftEncoderMsg.header.frame_id = LEFT_MOTOR_TF_LINK;
    rightEncoderMsg.header.frame_id = RIGHT_MOTOR_TF_LINK;
    imuMsg.header.frame_id = IMU_TF_LINK;

    leftMotor.start();
    rightMotor.start();

    imu.setup(IMU_I2C_ADDRESS);
    delay(1000);
    imu.calibrateAccelGyro();
    imu.calibrateMag();
}

void loop()
{
    // ROS update.
    nh.spinOnce();
    tryPublishEncoders();
    tryPublishIMU();

    // Sensors / actuators update.
    MotorController::FailCode code = leftMotor.tryUpdate();
    if (code == MotorController::FailCode::ENCODER_FAILURE)
        SAFETY_recoverI2C();

    code = rightMotor.tryUpdate();
    if (code == MotorController::FailCode::ENCODER_FAILURE)
        SAFETY_recoverI2C();

    delay(1);
}

// Param loaders.
void loadParamsToMotorControllers()
{
    MotorController::ControlParams params;

    if (!nh.getParam(WHEEL_MAX_ANGULAR_VELOCITY_PARAM, &params.maxVelocity))
        params.maxVelocity = 0;

    if (!nh.getParam(LOOP_RATE_MS_PARAM, (int *)&params.loopUpdateRate))
        params.loopUpdateRate = 0;

    float pidParams[3];
    if (!nh.getParam(LEFT_MOTOR_PID_PARAMS, pidParams, 3))
    {
        params.kp = 0;
        params.ki = 0;
        params.kd = 0;
    }
    leftMotor.setMotorParams(params);

    if (!nh.getParam(LEFT_MOTOR_PID_PARAMS, pidParams, 3))
    {
        params.kp = 0;
        params.ki = 0;
        params.kd = 0;
    }
    rightMotor.setMotorParams(params);
}

// Callbacks for subscribers.
void setpointsSubscriberCallback(const statek_msgs::Velocity &setpoints)
{
    // Don't disturb the service.
    if (SAFETY_serviceInProgress)
    {
        return;
    }

    rightMotor.setVelocity(setpoints.right);
    leftMotor.setVelocity(setpoints.left);
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
    imuMsg.orientation;

    imuMsg.angular_velocity;

    imuMsg.linear_acceleration.x = imu.getAccX();
    imuMsg.linear_acceleration.y = imu.getAccY();
    imuMsg.linear_acceleration.z = imu.getAccZ();

    imuPublisher.publish(&imuMsg);

    seq++;
    return;
}

void SAFETY_communicationFlowSupervisor()
{
    static unsigned long long previousCommunicationCheckTime = millis();

    unsigned long long now = millis();

    // Handle clock overflow.
    if ((now >= previousCommunicationCheckTime))
    {
        // Check if 100ms passed so we can publish encoders.
        if (now - previousCommunicationCheckTime >= 500)
        {
            // The flag was not reset by motors/vel_cmd subscriber
            // so treat it as broken communication flow and stop the motors.
            if (SAFETY_stopMotorsFlag)
            {
                rightMotor.setVelocity(0);
                leftMotor.setVelocity(0);
            }
            // The flag was reset by motors/vel_cmd callback
            // so communication flow is ok.
            else
            {
                SAFETY_stopMotorsFlag = true; // Set it again and wait for motors/vel_cmd to reset it hopefully.
            }
            previousCommunicationCheckTime = now;
        }
    }
    else
    {
        previousCommunicationCheckTime = now;
    }
}

void SAFETY_recoverI2C() {
    nh.logwarn("I2C stuck detected!");

    // SDA held low by the slave.
    if(digitalRead(D_SDA) == LOW){
        Wire.end();
        pinMode(D_SDA, OUTPUT);

        // Perform max 9 nine clock pulses
        // to unstuck the line.
        for(uint8_t i = 0; i < 9; i++){
            digitalWrite(D_SDA, HIGH);

            if(digitalRead(D_SDA) != LOW){
                break;
            }
        }

        if(digitalRead(D_SDA) == LOW)
            nh.logerror("Unable to recover I2C!");
        else
            nh.loginfo("I2C recovered.");

        // Restore I2C line and generate STOP condition.
        Wire.begin();
        Wire.endTransmission();
    }
}