#include "mbed.h"

#include "../include/ros_handlers/ros_handlers.hpp"
#include "../include/motor_controller/motor_controller.hpp"
#include "statek_msgs/Velocity.h"
#include "std_srvs/Empty.h"
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
 * @brief Callback fired when someone resuests motors/set_manual_control service. Sets motors into manual control mode.
 * 
 * @param req Unused.
 * @param res Unused.
 */
void setManualControl(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

/**
 * @brief Callback fired when someone resuests motors/set_manual_control service. Sets motors into automatic control mode.
 * 
 * @param req Unused.
 * @param res Unused.
 */
void setAutoControl(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

I2C i2c(D_SDA, D_SCL); //!< I2C to communicate with AM4096 encoders and MPU9250 IMU.
MotorController rightMotor({PA_0, PA_8, PA_9, PB_4}, i2c, 30, MotorController::Side::RIGHT, true); //!< Right motor control loop.
MotorController leftMotor({PA_1, PB_5, PC_7, PB_10}, i2c, 60, MotorController::Side::LEFT); //!< Left motor control loop.

statek_msgs::Velocity setpoints; //!< Setpoints in [rad/s] for both motors.
ros::Subscriber<statek_msgs::Velocity> setpointsSubscriber("motors/vel_cmd", &setpointsCallback); //!< Subscriber waiting for new setpoint values.
ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> manualControlService("motors/set_manual_control", &setManualControl);
ros::ServiceServer<std_srvs::EmptyRequest, std_srvs::EmptyResponse> automaticControlService("motors/set_automatic_control", &setAutoControl);

int main() {
    nh.initNode();
    nh.subscribe(setpointsSubscriber);
    nh.advertiseService(manualControlService);
    nh.advertiseService(automaticControlService);

    leftMotor.start();
    rightMotor.start();

    while (true) {
        nh.spinOnce();
        Thread::wait(5);
    }
}

void setpointsCallback(const statek_msgs::Velocity &setpoints){
    rightMotor.setVelocity(setpoints.right);
    leftMotor.setVelocity(setpoints.left);
}

void setManualControl(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    rightMotor.setControlMode(MotorController::ControlMode::MANUAL);
    leftMotor.setControlMode(MotorController::ControlMode::MANUAL);
}

void setAutoControl(const std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
    rightMotor.setControlMode(MotorController::ControlMode::AUTO);
    leftMotor.setControlMode(MotorController::ControlMode::AUTO);
}