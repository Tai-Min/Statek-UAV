# Real time hardware
All steps described here should be performed on **WSL 2** and **not** on Jetson Nano
## Setup ROS
Setup ROS again as described in [04_ros.md](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/04_ros.md) ._.

On the bright side, you will be able to use project's launch files such as Rviz visualization or
teleop nodes on your PC. To do so, set ROS_MASTER_URI
environmental variable as described in [00_environment_setup.md](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/00_environment_setup.md).
This will allow to perform supervisory control over UAV.

## Install gcc4mbed
As described in [here](https://github.com/adamgreen/gcc4mbed)
