# ROS
## Setting up ROS 
Install ROS Melodic Morenia as described [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

After that, setup your catkin workspace as described in [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

## Setting up ROS packages
Copy content of this repository's catkin_ws/src content into your catkin_ws/src folder.

Now, install third party packages:
* [ds4_driver](http://wiki.ros.org/ds4_driver)
* [ydlidar_ros](https://github.com/YDLIDAR/ydlidar_ros)
* [rosserial](http://wiki.ros.org/rosserial_mbed/Tutorials/rosserial_mbed%20Setup) (Skip gcc4mbed part as it does not work on Jetson Nano)

Now, in your catkin_ws folder run:
```
find . -type f -name "*.py" -print0 | xargs -0 chmod +x
```
To make all python nodes executable.

Edit cv_bridge's config:
```
sudo nano /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake
```
Search for lines:
```
if(NOT "include;/usr/include;/urs/include/opencv " STREQUAL " ")
...
set(_include_dirs "include;/usr/include;/urs/include/opencv")
```
And replace them with:
```
if(NOT "include;/usr/include;/usr/include/opencv4 " STREQUAL " ")
...
set(_include_dirs "include;/usr/include;/usr/include/opencv4")
```
Save file.

Lastly, run:
```
catkin_make 
```
inside you catkin workspace.

Previous: []() </br>
Next: []()