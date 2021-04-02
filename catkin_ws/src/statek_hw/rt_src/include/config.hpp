#pragma once

#define D_SDA PB_9
#define D_SCL PB_8

#define RIGHT_MOTOR_GPIO {PA_0, PA_8, PA_9, PB_4} // Enable, clockwise, counter clockwise, pwm.
#define RIGHT_MOTOR_ENCODER_I2C_ADDRESS 0x1E
#define RIGHT_MOTOR_ENCODER_TOPIC "motors/right/encoder"
#define RIGHT_MOTOR_TF_LINK "/wheels/right_front_link"
#define RIGHT_MOTOR_PID_PARAMS "~right_motor_pid"
#define RIGHT_MOTOR_PARAM_SERVICE "motors/right/set_params"

#define LEFT_MOTOR_GPIO {PA_1, PB_5, PC_7, PB_10}
#define LEFT_MOTOR_ENCODER_I2C_ADDRESS 0x3C
#define LEFT_MOTOR_ENCODER_TOPIC "motors/left/encoder"
#define LEFT_MOTOR_TF_LINK "/wheels/left_front_link"
#define LEFT_MOTOR_PID_PARAMS "~left_motor_pid"
#define LEFT_MOTOR_PARAM_SERVICE "motors/left/set_params"

#define WHEEL_MAX_ANGULAR_VELOCITY_PARAM "~wheel_max_angular_velocity"
#define LOOP_RATE_MS_PARAM "~loop_rate_ms"

#define VELOCITY_SETPOINTS_TOPIC "motors/vel_cmd"

#define VELOCITY_TEST_SERVICE "motors/max_velocity_test"
#define DIRECT_CONTROL_SERVICE "motors/set_direct_control"
#define PID_CONTROL_SERVICE "motors/set_pid_control"

#define IMU_I2C_ADDRESS 0x68
#define IMU_TOPIC "/imu"
#define IMU_TF_LINK "/imu"