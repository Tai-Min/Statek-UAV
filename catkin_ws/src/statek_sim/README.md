# statek_sim
Package with Statek's sim model / plugins and collection of Gazebo worlds.
* **models**
  * **statek** Statek's Gazebo model.
    * **params**
      * **TODO: <~stationary_velocity_threshold>** - Velocity threshold below which Statek is considered to be stationary. Fine tune this value to prevent noise from affecting some parts of the system.
      * **TODO: <~brake_velocity_threshold>** - Velocity difference between velocity cmds and measured velocities which should turn on brake lights.
    * **subscribed topics** 
      * **<statek_name>/vel_cmd_left** - Velocity setpoint for left motor.
      * **<statek_name>/vel_cmd_right** - Velocity setpoint for right motor.
      * **TODO: <statek_name>/front_lights** - Status of front lights. On "AUTO" On.
      * **TODO: <statek_name>/back_lights** - Status of back lights. On "AUTO" back lights mimics state of front lights.
      * **TODO: <statek_name>/left_blinker** - Status of left front and back blinkers. On "AUTO" OFF.
      * **TODO: <statek_name>/right_blinker** - Status of right front and back blinkers. On "Auto" OFF.
      * **TODO: <statek_name>/reversing_lights** - Status of reversing lights. On "AUTO" On when both velocity cmds are negative.
      * **TODO: <statek_name>/brake_lights** - Status of brake lights. On "AUTO" On when both velocity cmds are set to 0 and measured velocities are above <~stationary_velocity_threshold> or when difference between both velocity cmds and measured velocities are above <~brake_velocity_threshold>, Off otherwise.
    * **published topics**
      * **<statek_name>/encoder_raw_left** - Position and velocity readings from left encoder.
      * **<statek_name>/encoder_raw_right** - Position and velocity readings from right encoder.
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
  * **TODO: statek_lights_control** - Required to control Statek's lights in Gazebo.
    * **TODO: params**
* **worlds**
  * **simple_world.world** - Open simple world with some primitives placed around.
* **launch files**
  * **simple_world.launch** - Launch simple_world.world with Statek UAV spawned at origin.
  * **spawn_statek.launch** - Spawn statek in Gazebo.
    * **args** 
      * **name** - Name of the model to spawn. Used as namespace for topics, i.e statek_name/scan. Default is statek.
      * **TODO: x** - X world coordinate. Default is 0.
      * **TODO: y** - Y world coordinate. Default is 0.
      * **TODO: z** - Z world coordinate. Default is 0.
    * **results** - Launch file will spawn Statek uav with nodes / topics under **\<name>/** namespace.
