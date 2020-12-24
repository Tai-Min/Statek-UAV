# statek_rviz
Package with nodes required for teleoperation.
* **rviz**
  * **statek.rviz** - Basic robot view. For now requires that all launch files were run with default "statek_name" argument so rviz will subscribe to correct namespaces.

* **launch files**
  * **view.launch** - Run rviz with statek.rviz file.
    | args | description |
    |-|-|
    | statek_name | Used to create namespace for robot_description parameter. For now must be set to default value. Default value is statek. |

    | results |
    |-|
    | Opens Rviz with statek.rviz config file. |
