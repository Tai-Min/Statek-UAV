# statek_sim
Package with Statek's sim model / plugins and collection of Gazebo worlds.
* **models**
  * **statek** Statek's Gazebo model.
    | params | |
    |-|-|
    | <span style="color:#F6D82C">TODO: <~stationary_velocity_threshold></span> | Velocity threshold below which Statek is considered to be stationary. Fine tune this value to prevent noise from affecting some parts of the system. |
    | <span style="color:#F6D82C">TODO: <~brake_velocity_threshold></span> | Velocity difference between velocity cmds and measured velocities which should turn on brake lights. |

    | subscribed topics | |
    |-|-|
    | <statek_name>/vel_cmd_left | Velocity setpoint for left motor. |
    | <statek_name>/vel_cmd_right | Velocity setpoint for right motor. |
    | <span style="color:#F6D82C">TODO: <statek_name>/front_lights</span> | Status of front lights. On "AUTO" On. |
    | <span style="color:#F6D82C">TODO: <statek_name>/back_lights</span> | Status of back lights. On "AUTO" back lights mimics state of front lights. |
    | <span style="color:#F6D82C">TODO: <statek_name>/left_blinker</span> | Status of left front and back blinkers. On "AUTO" OFF. |
    | <span style="color:#F6D82C">TODO: <statek_name>/right_blinker</span> | Status of right front and back blinkers. On "Auto" OFF. |
    | <span style="color:#F6D82C">TODO: <statek_name>/reversing_lights</span> | Status of reversing lights. On "AUTO" On when both velocity cmds are negative. |
    | <span style="color:#F6D82C">TODO: <statek_name>/brake_lights</span> | Status of brake lights. On "AUTO" On when both velocity cmds are set to 0 and measured velocities are above <~stationary_velocity_threshold> or when difference between both velocity cmds and measured velocities are above <~brake_velocity_threshold>, Off otherwise. |

    | published topics | |
    |-|-|
    | <span style="color:#F6D82C">TODO: <statek_name>/encoder_raw_left</span> | Position and velocity readings from left encoder. |
    | <span style="color:#F6D82C">TODO: <statek_name>/encoder_raw_right</span> | Position and velocity readings from right encoder. |

* **plugins**
  * **statek_motor_control** - Required to control Statek's model in Gazebo.
    | params | |
    |-|-|
    | kp | Proportional gain of PID regulators for both motors. Default is 0.1. |
    | ki | Integral gain for PID regulators for both motors. Default is 0. |
    | kd | Derivative gain for PID regulators for both motors. Default is 0. |
    | left_back_joint | Name of left back revolute joint of the model.  Default is left_back. |
    | right_back_joint | Name of right back revolute joint of the model.  Default is right_back. |
    | left_front_joint | Name of left front revolute joint of the model.  Default is left_front. |
    | right_front_joint | Name of right front revolute joint of the model.  Default is right_front. |
    | <span style="color:#F6D82C">TODO: left_motor_topic_name</span> | Name of ROS topic for cmd commands for left motor. Default is vel_cmd_left. |
    | <span style="color:#F6D82C">TODO: right_motor_topic_name</span> | Name of ROS topic for cmd commands for right motor. Default is vel_cmd_right. |
    | <span style="color:#F6D82C">TODO: left_encoder_topic_name</span> | Name of ROS topic for encoder data. Default is encoder_raw_left. |
    | <span style="color:#F6D82C">TODO: right_encoder_topic_name</span> | Name of ROS topic for encoder data. Default is encoder_raw_right. |
    | <span style="color:#F6D82C">TODO: noise_power</span> | Noise power for encoder reading noise. Default is 0. |

  * **<span style="color:#F6D82C">TODO: statek_lights_control</span>** - Required to control Statek's lights in Gazebo.
    | <span style="color:#F6D82C">TODO: params</span> | |
    |-|-|
* **worlds**
  * **simple_world.world** - Open simple world with some primitives placed around.
* **launch files**
  * **simple_world.launch** - Launch simple_world.world with Statek UAV spawned at origin.
  * **spawn_statek.launch** - Spawn statek in Gazebo.
    | args | |
    |-|-|
    | name | Name of the model to spawn. Used as namespace for topics, i.e statek_name/scan. Default is statek. |
    | <span style="color:#F6D82C">TODO: x</span> | X world coordinate. Default is 0. |
    | <span style="color:#F6D82C">TODO: y</span> | Y world coordinate. Default is 0. |
    | <span style="color:#F6D82C">TODO: z</span> | Z world coordinate. Default is 0. |

    | results |
    |-|
    | Launch file will spawn Statek uav with nodes / topics under **\<name>/** namespace. |
