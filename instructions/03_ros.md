# ROS
## Install Ipopt and  MUMPS
Install Ipopt and MUMPS as described [here](https://coin-or.github.io/Ipopt/INSTALL.html) (Perform MUMPS steps in home directory).
For Ipopt's configure step type:
```
ldconfig
export CPLUS_INCLUDE_PATH=/usr/include/mpi:$CPLUS_INCLUDE_PATH
export LIBRARY_PATH=/usr/lib/openmpi/lib:$LIBRARY_PATH
../configure --with-mumps-cflags="-I$HOME/ThirdParty-Mumps/MUMPS/include -DCMAKE_CXX_COMPILER=/usr/bin/mpic++" --with-mumps-lflags="-pthread -L/usr//lib -L/usr/lib/aarch64-linux-gnu/openmpi/lib -lmpi_cxx -lcoinmumps" --without-hsl
```

## Setting up ROS 
Install ROS Melodic Morenia as described [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

After that, setup your catkin workspace as described in [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

## Setting up ROS packages
Copy content of this repository's catkin_ws/src content into your catkin_ws/src folder.

Now, install third party packages:
* [ds4_driver](http://wiki.ros.org/ds4_driver)
* [ydlidar_ros](https://github.com/YDLIDAR/ydlidar_ros)
* [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver)
* [rosserial](https://github.com/ros-drivers/rosserial) (select melodic branch to clone)
* [tuw_multi_robot](https://github.com/tuw-robotics/tuw_multi_robot) (select melodic branch to clone)
* [tuw_msgs](https://github.com/tuw-robotics/tuw_msgs) (select melodic branch to clone)
* [tuw_geometry](https://github.com/tuw-robotics/tuw_geometry) (select melodic branch to clone)

cd into rosserial:
```
roscd rosserial_python/src/rosserial_python
```
open SerialClient.py and repleace:
```
import queue
```
with:
```
import Queue as queue
```

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

Previous: [OS preparation](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/02_os_preparation.md) </br>
Next: [Remote environment setup](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/04_remote_environment_setup.md)
