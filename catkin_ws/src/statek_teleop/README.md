# statek_teleop
Package with nodes required for teleoperation.
* **nodes**
  * **statek_teleop_raw_ds4.py** - Translates Dualshock 4 joy inputs into raw velocity commands for motors of Statek UAV. Requires [ds4_driver](http://wiki.ros.org/ds4_driver).
  
    | params | description |
    |-|-|
    | statek_name | Name of Statek UAV to control. |
    | gamepad_name | Name of gamepad to listen to. |

    | subscribed topics | description |
    |-|-|
    | <gamepad_name>/status | Status of the gamepad. |

    | published topics | description |
    |-|-|
    | <statek_name>/vel_cmd_left | Velocity setpoint for left motor. |
    | statek_name>/vel_cmd_right | Velocity setpoint for right motor. |

* **launch files**
  * **statek_teleop_raw_ds4.launch** - Launch nodes required for teleoperation using Dualshock 4 and raw velocity commands. 
    | args | description |
    |-|-|
    | statek_name | Name of Statek UAV to control. Also used to create namespaces for topics. Default is statek. |

    | results |
    |-|
    | Launch file will create nodes / topics for controller under **<statek_name>_controller/** namespace and teleop nodes / topics under **<statek_name>_teleop/** namespace. |
