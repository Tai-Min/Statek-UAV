# statek_config
Package with Statek's config and robot description files.
* **urdf**
  * **statek.urdf** Statek's robot description file used for Gazebo and Rviz.

    | args | description |
    |-|-|
    | statek_name | Name of current Statek UAV. Used to create namespace for Gazebo plugins and tf links. Default is statek. |

    | subscribed topics (when run in Gazebo) | |
    |-|-|
    | <statek_name>/vel_cmd_left | Velocity setpoint for left motor. |
    | <statek_name>/vel_cmd_right | Velocity setpoint for right motor. |
