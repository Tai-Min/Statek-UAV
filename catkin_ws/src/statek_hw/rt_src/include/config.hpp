#pragma once

#define D_SDA PB9
#define D_SCL PB8

#define RIGHT_MOTOR_GPIO   \
    {                      \
        PA0, PA8, PA9, PB4 \
    } // Enable, clockwise, counter clockwise, pwm.
#define RIGHT_MOTOR_ENCODER_I2C_ADDRESS 0x1E
#define RIGHT_MOTOR_ENCODER_TOPIC "motors/right/encoder"
#define RIGHT_MOTOR_TF_LINK "/wheels/right_front_link"
#define RIGHT_MOTOR_PID_PARAMS "~right_motor_pid"
#define RIGHT_MOTOR_PARAM_SERVICE "motors/right/set_params"

#define LEFT_MOTOR_GPIO     \
    {                       \
        PA1, PB5, PC7, PB10 \
    }
#define LEFT_MOTOR_ENCODER_I2C_ADDRESS 0x3C
#define LEFT_MOTOR_ENCODER_TOPIC "motors/left/encoder"
#define LEFT_MOTOR_TF_LINK "/wheels/left_front_link"
#define LEFT_MOTOR_PID_PARAMS "~left_motor_pid"
#define LEFT_MOTOR_PARAM_SERVICE "motors/left/set_params"

#define WHEEL_MAX_ANGULAR_VELOCITY_PARAM "~wheel_max_angular_velocity"
#define LOOP_RATE_MS_PARAM "~loop_rate_ms"

#define VELOCITY_SETPOINTS_TOPIC "motors/vel_cmd"

#define VELOCITY_TEST_SERVICE "motors/max_velocity_test"
#define STEP_RESPONSE_IDENTIFICATION_SERVICE "motors/step_response_identification"
#define DIRECT_CONTROL_SERVICE "motors/set_direct_control"
#define PID_CONTROL_SERVICE "motors/set_pid_control"

#define IMU_I2C_ADDRESS 0x68
#define IMU_TOPIC "imu"
#define IMU_TF_LINK "/imu"
#define IMU_CALIBRATION_SERVICE "imu/calibrate"
#define IMU_PARAM_SERVICE "imu/set_params"

#define ODOM_TOPIC "odom"
#define ODOM_TF_LINK "/odom"
#define ODOM_CHILD_LINK "/base_footprint"
#define ODOM_PARAM_SERVICE "odom/set_params"
