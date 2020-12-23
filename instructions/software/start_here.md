# Software preparation

## Jetson Nano OS preparation
Install Jetson Nano OS as described [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit).

After that, follow [OS preparation](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/software/OS_preparation.md) part and when completed, return here to setup ROS environment.

## Setting up ROS 
Install ROS Melodic Morenia as described [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

After that, setup your catkin workspace as described in [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

## Setting up ROS packages
Copy content of this repository's catkin_ws/src content into your catkin_ws/src folder and run :
```
catkin_make 
```
inside you catkin workspace.

Now, install third party packages:
* [ds4_driver](http://wiki.ros.org/ds4_driver)
* [ydlidar_ros](https://github.com/YDLIDAR/ydlidar_ros)

Now, in your catkin_ws folder run:
```
find . -type f -name "*.py" -print0 | xargs -0 chmod +x
```
To make all python nodes executable.
