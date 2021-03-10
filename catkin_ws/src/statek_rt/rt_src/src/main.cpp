#include "mbed.h"

#include "../include/ros_handlers/ros_handlers.hpp"
#include "../include/motor_controller/motor_controller.hpp"
#include "statek_msgs/Velocity.h"
#include "statek_msgs/VelocityTest.h"
#include "std_srvs/Empty.h"

#define D_SDA PB_9
#define D_SCL PB_8

/**
 * @brief Callback fired on new message on motors/vel_cmd topic. Sets new setpoints for both motors.
 * 
 * @param setpoints Setpoints to set on both motors.
 */
void setpointsCallback(const statek_msgs::Velocity &setpoints);

/**
 * @brief Callback fired on motors/velocity_test service. Returns max possible velocity that can be set to both motors.
 * 
 * @param req Request - For how long to test the motors.
 * @param res Response - Maximum possible velocity.
 */
void velocityTest(const statek_msgs::VelocityTestRequest &req, statek_msgs::VelocityTestResponse &res);

/**
 * @brief Callback fired when someone resuests motors/set_manual_control service. Sets motors into manual control mode.
 * 
 * @param req Unused.
 * @param res Unused.
 */
void setManualControl(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

/**
 * @brief Callback fired when someone resuests motors/set_manual_control service. Sets motors into automatic control mode.
 * 
 * @param req Unused.
 * @param res Unused.
 */
void setAutoControl(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

/**
 * @brief Safety function. Stops the motors in case of broken flow on motors/vel_cmd topic.
 */
void SAFETY_communicationFlowSupervisor();

I2C i2c(D_SDA, D_SCL);                                                                             //!< I2C to communicate with AM4096 encoders and MPU9250 IMU.
MotorController rightMotor({PA_0, PA_8, PA_9, PB_4}, i2c, 30, MotorController::Side::RIGHT, true); //!< Right motor control loop.
MotorController leftMotor({PA_1, PB_5, PC_7, PB_10}, i2c, 60, MotorController::Side::LEFT);        //!< Left motor control loop.

statek_msgs::Velocity setpoints;                                                                  //!< Setpoints in [rad/s] for both motors.
ros::Subscriber<statek_msgs::Velocity> setpointsSubscriber("motors/vel_cmd", &setpointsCallback); //!< Subscriber waiting for new setpoint values.
ros::ServiceServer<statek_msgs::VelocityTestRequest, statek_msgs::VelocityTestResponse> velocityTestService("motors/velocity_test", &velocityTest);
ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> manualControlService("motors/set_manual_control", &setManualControl);     //!< Service to set motors into manual control.
ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> automaticControlService("motors/set_automatic_control", &setAutoControl); //!< Service to set motors into automatic control.

volatile bool SAFETY_stopMotorsFlag = false; //!< Used to stop the motors by SAFETY_communicationFlowSupervisor function. Should be reset every time a new message in motors/vel_cmd arrives.
Ticker communicationFlowSupervisorTicker;    //!< Interval to invoke SAFETY_communicationFlowSupervisor callback.

int main()
{
    nh.initNode();

    leftMotor.start();
    rightMotor.start();

    nh.subscribe(setpointsSubscriber);
    nh.advertiseService(velocityTestService);
    nh.advertiseService(manualControlService);
    nh.advertiseService(automaticControlService);

    communicationFlowSupervisorTicker.attach(callback(&SAFETY_communicationFlowSupervisor), 0.5f);

    while (true)
    {
        nh.spinOnce();
        Thread::wait(5);
    }
}

void setpointsCallback(const statek_msgs::Velocity &setpoints)
{
    rightMotor.setVelocity(setpoints.right);
    leftMotor.setVelocity(setpoints.left);
    SAFETY_stopMotorsFlag = false; // A message arrived so don's stop the motors.
}

void velocityTest(const statek_msgs::VelocityTestRequest &req, statek_msgs::VelocityTestResponse &res)
{
    MotorController::ControlMode previousMode = rightMotor.getControlMode(); // Restore previous mode after test.

    // Begin the test.
    rightMotor.setControlMode(MotorController::ControlMode::VELOCITY_TEST);
    leftMotor.setControlMode(MotorController::ControlMode::VELOCITY_TEST);

    // Let it do it's job for some time.
    Thread::wait(req.time);

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
    res.velocity = maxVel;
}

void setManualControl(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    rightMotor.setControlMode(MotorController::ControlMode::MANUAL);
    leftMotor.setControlMode(MotorController::ControlMode::MANUAL);
}

void setAutoControl(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    rightMotor.setControlMode(MotorController::ControlMode::AUTO);
    leftMotor.setControlMode(MotorController::ControlMode::AUTO);
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