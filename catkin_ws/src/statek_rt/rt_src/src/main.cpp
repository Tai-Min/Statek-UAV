#include <mbed.h>
#include <statek_msgs/Velocity.h>
#include <statek_msgs/VelocityTest.h>
#include <std_srvs/Empty.h>

#include "../config.hpp"
#include "../include/ros_handlers/ros_handlers.hpp"
#include "../include/motor_controller/motor_controller.hpp"

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
void maxVelocityTestServiceCallback(const statek_msgs::VelocityTestRequest &req, statek_msgs::VelocityTestResponse &res);

/**
 * @brief Callback fired when someone requests motors/set_direct_control service. Sets motors into direct control mode.
 * 
 * @param req Unused.
 * @param res Unused.
 */
void setDirectControlServiceCallback(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

/**
 * @brief Callback fired when someone requests motors/set_state_feedback_control service. Sets motors into state feedback loop control mode.
 * 
 * @param req Unused.
 * @param res Unused.
 */
void setStateFeedbackControlServiceCallback(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

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
                               RIGHT_MOTOR_TF_LINK); //!< Right motor controller.
    MotorController leftMotor(LEFT_MOTOR_GPIO,
                              i2c, LEFT_MOTOR_ENCODER_I2C_ADDRESS,
                              LEFT_MOTOR_ENCODER_RAW_TOPIC,
                              LEFT_MOTOR_TF_LINK, true); //!< Left motor controller.

    statek_msgs::Velocity setpoints;                                                                                                                                          //!< Setpoints in [rad/s] for both motors.
    ros::Subscriber<statek_msgs::Velocity> setpointsSubscriber(VELOCITY_SETPOINTS_TOPIC, &setpointsSubscriberCallback);                                                       //!< Subscriber waiting for new setpoint values.
    ros::ServiceServer<statek_msgs::VelocityTestRequest, statek_msgs::VelocityTestResponse> maxVelocityTestService(VELOCITY_TEST_SERVICE, &maxVelocityTestServiceCallback);   //!< Service to perform max velocity test.
    ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> directControlService(DIRECT_CONTROL_SERVICE, &setDirectControlServiceCallback);                       //!< Service to set motors into manual control.
    ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> stateFeedbackControlService(STATE_FEEDBACK_CONTROL_SERVICE, &setStateFeedbackControlServiceCallback); //!< Service to set motors into state feedback control.

    volatile bool SAFETY_stopMotorsFlag = false;     //!< Used to stop the motors by SAFETY_communicationFlowSupervisor function. Should be reset every time a new message in motors/vel_cmd arrives.
    Ticker SAFETY_communicationFlowSupervisorTicker; //!< Interval to invoke SAFETY_communicationFlowSupervisor callback.
}

int main()
{
    nh.initNode();

    // Start motor threads along with their ROS stuff.
    leftMotor.start();
    rightMotor.start();

    nh.subscribe(setpointsSubscriber);
    nh.spinOnce();

    nh.advertiseService(maxVelocityTestService);
    nh.spinOnce();

    nh.advertiseService(directControlService);
    nh.spinOnce();

    nh.advertiseService(stateFeedbackControlService);
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

void maxVelocityTestServiceCallback(const statek_msgs::VelocityTestRequest &req, statek_msgs::VelocityTestResponse &res)
{
    // Don't run the test when another callback is doing stuff.
    if(serviceInProgress){
        res.velocity = -1;
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

    // Save it locally.
    rightMotor.setMaxVelocity(maxVel);
    leftMotor.setMaxVelocity(maxVel);

    // Finally, respond to the caller.
    res.velocity = maxVel;

    serviceInProgress = false;
}

void setDirectControlServiceCallback(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    rightMotor.setControlMode(MotorController::ControlMode::DIRECT);
    leftMotor.setControlMode(MotorController::ControlMode::DIRECT);
}

void setStateFeedbackControlServiceCallback(const std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    rightMotor.setControlMode(MotorController::ControlMode::STATE_FEEDBACK);
    leftMotor.setControlMode(MotorController::ControlMode::STATE_FEEDBACK);
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