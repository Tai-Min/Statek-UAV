#include <mbed.h>
#include <statek_msgs/Velocity.h>
#include <statek_msgs/RunVelocityTest.h>
#include <statek_msgs/SetMotorParams.h>
#include <std_srvs/Trigger.h>

#include "../config.hpp"
#include "../include/ros_handlers/ros_handlers.hpp"
#include "../include/motor_controller/motor_controller.hpp"
#include "../include/clock/clock.hpp"

/**
 * @brief Callback fired on new message on motors/vel_cmd topic. Sets new setpoints for both motors.
 * 
 * @param setpoints Setpoints to set on both motors.
 */
void setpointsSubscriberCallback(const statek_msgs::Velocity &setpoints);

/**
 * @brief Callback fired on motors/max_velocity_test service. Returns max possible velocity that can be set to both motors.
 * 
 * @param req Request - For how long to test the motors.
 * @param res Response - Maximum possible velocity.
 */
void maxVelocityTestServiceCallback(const statek_msgs::RunVelocityTestRequest &req, statek_msgs::RunVelocityTestResponse &res);

/**
 * @brief Callback fired when someone requests motors/set_direct_control service. Sets motors into direct control mode.
 * 
 * @param req Unused.
 * @param res Set to true if service succeed.
 */
void setDirectControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

/**
 * @brief Callback fired when someone requests motors/set_pid_control service. Sets motors into closed loop control mode.
 * 
 * @param req Unused.
 * @param res Set to true if service succeed.
 */
void setClosedLoopControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

/**
 * @brief Set given params to left motor.
 * 
 * @param req Request - Params to set.
 * @param res Response - Response to the called.
 */
void setLeftMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);

/**
 * @brief Set given params to right motor.
 * 
 * @param req Request - Params to set.
 * @param res Response - Response to the called.
 */
void setRightMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res);

/**
 * @brief Safety function. Stops the motors in case of broken communication flow on motors/vel_cmd topic.
 */
void SAFETY_communicationFlowSupervisor();

namespace
{
    I2C i2c(D_SDA, D_SCL); //!< I2C to communicate with AM4096 encoders and MPU9250 IMU.
    MotorController rightMotor(RIGHT_MOTOR_GPIO,
                               i2c, RIGHT_MOTOR_ENCODER_I2C_ADDRESS,
                               RIGHT_MOTOR_ENCODER_RAW_TOPIC,
                               RIGHT_MOTOR_KP_PARAM,
                               RIGHT_MOTOR_KI_PARAM,
                               RIGHT_MOTOR_KD_PARAM,
                               RIGHT_MOTOR_TF_LINK); //!< Right motor controller.
    MotorController leftMotor(LEFT_MOTOR_GPIO,
                              i2c, LEFT_MOTOR_ENCODER_I2C_ADDRESS,
                              LEFT_MOTOR_ENCODER_RAW_TOPIC,
                              LEFT_MOTOR_KP_PARAM,
                              LEFT_MOTOR_KI_PARAM,
                              LEFT_MOTOR_KD_PARAM,
                              LEFT_MOTOR_TF_LINK, true); //!< Left motor controller.

    statek_msgs::Velocity setpoints; //!< Setpoints in [rad/s] for both motors.
    ros::Subscriber<statek_msgs::Velocity>
        setpointsSubscriber(VELOCITY_SETPOINTS_TOPIC, &setpointsSubscriberCallback); //!< Subscriber waiting for new setpoint values.
    ros::ServiceServer<statek_msgs::RunVelocityTestRequest, statek_msgs::RunVelocityTestResponse>
        maxVelocityTestService(VELOCITY_TEST_SERVICE, &maxVelocityTestServiceCallback); //!< Service to perform max velocity test.
    ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
        directControlService(DIRECT_CONTROL_SERVICE, &setDirectControlServiceCallback); //!< Service to set motors into direct control.
    ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
        closedLoopControlService(PID_CONTROL_SERVICE, &setClosedLoopControlServiceCallback); //!< Service to set motors into pid control.
    ros::ServiceServer<statek_msgs::SetMotorParamsRequest, statek_msgs::SetMotorParamsResponse>
        setLeftMotorParamsService(LEFT_MOTOR_PARAM_SERVICE, &setLeftMotorParamsCallback);
    ros::ServiceServer<statek_msgs::SetMotorParamsRequest, statek_msgs::SetMotorParamsResponse>
        setrightMotorParamsService(RIGHT_MOTOR_PARAM_SERVICE, &setRightMotorParamsCallback);

    volatile bool SAFETY_stopMotorsFlag = false;     //!< Used to stop the motors by SAFETY_communicationFlowSupervisor function. Should be reset every time a new message in motors/vel_cmd arrives.
    Ticker SAFETY_communicationFlowSupervisorTicker; //!< Interval to invoke SAFETY_communicationFlowSupervisor callback.
}

int main()
{
    clockStart();

    i2c.frequency(100000);

    nh.initNode();

    // Start motor threads along with their ROS stuff.
    leftMotor.start();
    rightMotor.start();

    // Start services shared by some objects.
    nh.subscribe(setpointsSubscriber);
    nh.spinOnce();

    nh.advertiseService(maxVelocityTestService);
    nh.spinOnce();

    nh.advertiseService(directControlService);
    nh.spinOnce();

    nh.advertiseService(closedLoopControlService);
    nh.spinOnce();

    // Start safety measurements
    SAFETY_communicationFlowSupervisorTicker.attach(callback(&SAFETY_communicationFlowSupervisor), 0.5f);

    while (true)
    {
        nh.spinOnce();
        Thread::wait(MAIN_THREAD_UPDATE_INTERVAL);
    }
}

void setpointsSubscriberCallback(const statek_msgs::Velocity &setpoints)
{
    rightMotor.setVelocity(setpoints.right);
    leftMotor.setVelocity(setpoints.left);
    SAFETY_stopMotorsFlag = false; // A message arrived so don's stop the motors.
}

void maxVelocityTestServiceCallback(const statek_msgs::RunVelocityTestRequest &req, statek_msgs::RunVelocityTestResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if (serviceInProgress)
    {
        res.success = false;
        return;
    }

    serviceInProgress = true;

    MotorController::ControlMode previousMode = rightMotor.getControlMode(); // Restore previous mode after test.

    // Begin the test.
    rightMotor.setControlMode(MotorController::ControlMode::MAX_VELOCITY_TEST);
    leftMotor.setControlMode(MotorController::ControlMode::MAX_VELOCITY_TEST);

    // Let it do it's job for some time.
    // But also keep communication with ROS alive.
    for (unsigned int i = 0; i < req.time; i += 10)
    {
        SAFETY_stopMotorsFlag = false; // Also reset this flag so motor's won't stop.
        nh.spinOnce();
        Thread::wait(10);
    }

    // Restore previous mode.
    rightMotor.setControlMode(previousMode);
    leftMotor.setControlMode(previousMode);

    // Treat mean velocity as maximum velocity to reject noise etc.
    double rightMaxVel = rightMotor.getTestedMeanVelocity();
    double leftMaxVel = leftMotor.getTestedMeanVelocity();

    // Set lower value as max possible velocity so both motors will be able to achieve it.
    double maxVel = rightMaxVel;
    if (leftMaxVel < rightMaxVel)
        maxVel = leftMaxVel;

    // Finally, respond to the caller.
    res.success = true;
    res.velocity = maxVel;

    serviceInProgress = false;
}

void setDirectControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    if (serviceInProgress)
    {
        res.success = false;
        return;
    }
    serviceInProgress = true;

    rightMotor.setControlMode(MotorController::ControlMode::DIRECT);
    leftMotor.setControlMode(MotorController::ControlMode::DIRECT);
    res.success = true;

    serviceInProgress = false;
}

void setClosedLoopControlServiceCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    if (serviceInProgress)
    {
        res.success = false;
        return;
    }
    serviceInProgress = true;

    rightMotor.setControlMode(MotorController::ControlMode::PID_CONTROL);
    leftMotor.setControlMode(MotorController::ControlMode::PID_CONTROL);
    res.success = true;

    serviceInProgress = false;
}

void setLeftMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res){
    leftMotor.setMotorParamsCallback(req, res);
}

void setRightMotorParamsCallback(const statek_msgs::SetMotorParamsRequest &req, statek_msgs::SetMotorParamsResponse &res){
    rightMotor.setMotorParamsCallback(req, res);
}

void SAFETY_communicationFlowSupervisor()
{
    // The flag was not reset by motors/vel_cmd subscriber
    // so treat it as broken communication flow and stop the motors.
    if (SAFETY_stopMotorsFlag)
    {
        rightMotor.SAFETY_stopMotor();
        leftMotor.SAFETY_stopMotor();
    }
    // The flag was reset by motors/vel_cmd callback
    // so communication flow is ok.
    else
    {
        SAFETY_stopMotorsFlag = true; // Set it again and wait for motors/vel_cmd to reset it hopefully.
    }
}