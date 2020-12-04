Catkin packages for Statek UAV.

* **statek_playground** </br>
  Package with various launch files to play with simulated or real UAV.
  * **launch files**
    * **simple_world_sim_raw_ds4.launch** - Control simulated Statek's motors in simple Gazebo world using Dualshock 4 joysticks. Left joy for left motor and right joy for right motor.
      * **args**
        * **name** - Name of model to spawn. Used to create namespaces for control stack.
* **statek_sim** </br>
  Package with Statek's sim model / plugins and collection of Gazebo worlds.
  * **models**
    * **statek** Statek's Gazebo model.
  * **plugins**
    * **statek_motor_control** - Required to control Statek's model in Gazebo.
      * **params**
        * **kp** - Proportional gain of PID regulators for both motors. Default is 0.1.
        * **ki** - Integral gain for PID regulators for both motors. Default is 0.
        * **kd** - Derivative gain for PID regulators for both motors. Default is 0.
        * **left_back_joint** - Name of left back revolute joint of the model.  Default is left_back.
        * **right_back_joint** - Name of right back revolute joint of the model.  Default is right_back.
        * **left_front_joint** - Name of left front revolute joint of the model.  Default is left_front.
        * **right_front_joint** - Name of right front revolute joint of the model.  Default is right_front.
        * **TODO: left_motor_topic_name** - Name of ROS topic for cmd commands for left motor. Default is vel_cmd_left.
        * **TODO: right_motor_topic_name** - Name of ROS topic for cmd commands for right motor. Default is vel_cmd_right.
        * **TODO: left_encoder_topic_name** - Name of ROS topic for encoder data. Default is encoder_raw_left.
        * **TODO: right_encoder_topic_name** - Name of ROS topic for encoder data. Default is encoder_raw_right.
        * **TODO: noise_power** - Noise power for encoder reading noise. Default is 0.
  * **worlds**
    * **simple_world.world** - Simple world with some primitives placed around.
  * **launch files**
    * **simple_world.launch** - Launch simple_world.world with Statek UAV spawned at origin.
    * **spawn_statek.launch** - Spawn statek in Gazebo.
      * **args** 
        * **name** - Name of the model to spawn. Used as namespace for topics, i.e statek_name/scan. Default is statek.
        * **TODO: x** - X world coordinate.
        * **TODO: y** - Y world coordinate.
        * **TODO: z** - Z world coordinate.
        
* **statek_teleop** </br>
  Package with nodes required for teleoperation.
  * **nodes**
    * **statek_teleop_raw_ds4.py** - Translates Dualshock 4 joy inputs into raw velocity commands for motors of Statek UAV. Requires [ds4_driver](http://wiki.ros.org/ds4_driver).
      * **params**
        * statek_name - Name of Statek UAV to control.
        * gamepad_name - Name of gamepad to listen to.
      * **subscribed topics**
      * **published topics**
