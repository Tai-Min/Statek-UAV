# statek_teleop
Package with nodes required for teleoperation.
* **nodes**
  * **statek_teleop_raw_ds4.py** - Translates Dualshock 4 joy inputs into raw velocity commands for motors of Statek UAV.
  
    | params | description |
    |-|-|
    | statek_name | Namespace of Statek UAV to control. Defaults to statek |
    | gamepad_name | Namespace of the gamepad to listen to. Defaults to ds4. |

    | subscribed topics | description |
    |-|-|
    | <gamepad_name>/status | Status of the gamepad. |

    | published topics | description |
    |-|-|
    | <statek_name>/vel_cmd_left | Velocity setpoint for left motor. |
    | <statek_name>/vel_cmd_right | Velocity setpoint for right motor. |

* **launch files**
  * **statek_teleop_raw_ds4.launch** - Launch nodes required for teleoperation using Dualshock 4 and raw velocity commands. 
    | args | description |
    |-|-|
    | statek_name | Name of Statek UAV to control. Also used to create namespaces for topics. Default is statek. |
    | gamepad_addr | Address of the gamepad. By default it connects to first found gamepad. |

    | results |
    |-|
    | Launch file will listen to status of firstly found gamepad and publish it's joystick info into **<statek_name>/vel_cmd_left** and **<statek_name>/vel_cmd_right** topics. |
