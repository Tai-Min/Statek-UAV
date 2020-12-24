# statek_playground
Package with various launch files to play with simulated or real UAV.
* **launch files**
  * **simple_world_sim_raw_ds4.launch** - Control simulated Statek's motors in simple Gazebo world using Dualshock 4 joysticks. Left joy for left motor and right joy for right motor. </br>
  
    | args | description |
    |-|-|
    | statek_name | Name of model to spawn. Used to create namespaces for control stack. Default is statek. |
    | gamepad_addr | Address of the gamepad. By default connects to first found gamepad. |

    | result |
    |-|
    | Launch file will create controller nodes / topics under **\<name>_controller/** namespace,  teleop nodes / topics under **\<name>_teleop/** namespace and Statek's nodes / topics under **\<name>/** namespace. |
