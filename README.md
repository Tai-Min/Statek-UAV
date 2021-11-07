# Statek-UAV
ROS based UAV, Master's degree final project.

This repository consists of CAD files for the chassis, CAD files for electronic boards, catkin workspace for all ROS nodes created for the project and some additional scripts used during development.

The core part of this project is the real, 3D printed vehicle along with all the hardware attached to it (Jetson Nano B01 for main controller and STM32f103RBT6 for real time core). 

The software part of this project can be split into real time (closed loop wheel control), short term navigation (movement between two GPS waypoints) and long term navigation (navigation between series of GPS waypoints).
During the project's development, my main focus was on short term navigation, which is also the biggest and most complicated part of the project. Short term navigation stack
consists of static local map generator, dynamic local map generator (human leg detector mostly based on U-Net fully convolutional neural network), Voronoi graph generator, A* path planner and non linear model predictive controller for navigation.

For more info see [this arcitle](https://www.hackster.io/Tai-Min/statek-uav-ff225e).

**Just to warn you:** If you want to use this project somehow, use it as reference only. It is NOT meant to be assembled as it contains some design flaws both in hardware
and software that I don't intent to fix in this iteration of the project as code in this repository will stay in unchanged since the moment I handed over my thesis.
Most of the files in the "instructions" folder consists of the steps I've did to setup my hardware and Ipopt solver. Those might not work for you/could be dangerous (I might've abused sudo a bit) so use them at your own risk. The code quality depends on the time I had untill the deadline to hand over my thesis. In my opinion it should be fine for the most part, well documented and easy to follow but judge for yourself.

**Disclaimer:** I've tried to credit all authors of 3D files used in the project. All of the models had permissive license allowing me to use them.
If you see your work here without being listed as the author or you just don't want to have your files used here - open an issue on this project and I'll fix any problems.

The files in this folder have the following structure:
* CAD
  * detailed - Inventor files for the 3D printing and assembling.
  * low_poly - Inventor files for simple low poly model for Gazebo and Rviz.
* catkin_ws/src
  * statek - Metapackage for the project
  * statek_calibrate - Calibration algorithms for cameras, GPS Kalman filter, IMU, Motor's PI controller autotuning. 
  * statek_config - Config files for the robot, including transformations, urdf and parameters for ROS nodes.
  * statek_hw - code to launch hardware based nodes (lidar, rosserial, cameras etc.) along with source code for real time controller.
  * statek_map - Mapping algorithms (dynamic local mapper, voronoi generator and GPS to hyperplane casting).
  * statek_ml - Leg detector based mostly on U-net fully convolutional neural network
  * statek_nav - Navigation algorithms (NMPC and GPS waypoint follower script).
  * statek_plan - A* path planner for short term navigation.
  * statek_playground - Simple launch nodes to play with the hardware.
  * statek_rviz - Rviz visualization files.
  * statek_sim - Gazebo simulation files.
  * statek_srv - Server files to communicate with web application (really rough draft).
  * statek_teleop - Teleoperation nodes for Dualshock 4 gamepad.
* electronics
  * AM4096_encoder - KiCad's files for AM4096 encoder board.
  * Nucleo_screw_shield - KiCad's files for screw shield that fits morpho connector.
  * button_panel - Unused in the project.
  * connections - Electrical connections of the project.
* instructions - Rough draft of what I did to setup the robot.
* media - Contains camera's calibration chessboard.
* thesis_scripts - Some files used to add more value to the project.

![alt text](https://hackster.imgix.net/uploads/attachments/1331930/_jCfJIjiPkr.blob?auto=compress%2Cformat&w=900&h=675&fit=min "Title")
