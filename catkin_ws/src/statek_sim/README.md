# statek_sim
Package with Statek's sim model / plugins and collection of Gazebo worlds.
* **models**
  * **statek** Statek's Gazebo model.

    | subscribed topics | |
    |-|-|
    | <statek_name>/vel_cmd_left | Velocity setpoint for left motor. |
    | <statek_name>/vel_cmd_right | Velocity setpoint for right motor. |

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

* **worlds**
  * **simple_world.world** - Open simple world with some primitives placed around.
* **launch files**
  * **simple_world.launch** - Launch simple_world.world with Statek UAV spawned at origin.
  * **spawn_statek.launch** - Spawn statek in Gazebo.
    | args | |
    |-|-|
    | name | Name of the model to spawn. Used as namespace for topics, i.e statek_name/scan. Default is statek. |
    | x | X world coordinate. Default is 0. |
    | y | Y world coordinate. Default is 0. |
    | z | Z world coordinate. Default is 0. |

    | results |
    |-|
    | Launch file will spawn Statek uav with nodes / topics under **\<name>/** namespace. |
