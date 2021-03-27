#pragma once

// I2C config.
#define D_SDA PB_9
#define D_SCL PB_8

#define MAIN_THREAD_UPDATE_INTERVAL 5 // In milliseconds.

#define RIGHT_MOTOR_GPIO {PA_0, PA_8, PA_9, PB_4} // Enable, clockwise, counter clockwise, pwm.
#define RIGHT_MOTOR_ENCODER_I2C_ADDRESS 30
#define RIGHT_MOTOR_ENCODER_RAW_TOPIC "motors/right/encoder"
#define RIGHT_MOTOR_TF_LINK "/wheels/right_front_link"

#define LEFT_MOTOR_GPIO {PA_1, PB_5, PC_7, PB_10}
#define LEFT_MOTOR_ENCODER_I2C_ADDRESS 60
#define LEFT_MOTOR_ENCODER_RAW_TOPIC "motors/left/encoder"
#define LEFT_MOTOR_TF_LINK "/wheels/left_front_link"

#define VELOCITY_SETPOINTS_TOPIC "motors/vel_cmd"

#define VELOCITY_TEST_SERVICE "motors/max_velocity_test"
#define DIRECT_CONTROL_SERVICE "motors/set_direct_control"
#define STATE_FEEDBACK_CONTROL_SERVICE "motors/set_state_feedback_control"
