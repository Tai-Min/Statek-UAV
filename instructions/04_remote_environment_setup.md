# Remote environment setup

## Setup WSL 2
### Setup Ubuntu subsystem for windows
https://docs.microsoft.com/en-us/windows/wsl/install-win10

### Then setup GUI for WSL 2
https://medium.com/@japheth.yates/the-complete-wsl2-gui-setup-2582828f4577

**Note:** Always start VcXsrv with -nowgl parameter and access control disabled.

### Establish connection with UAV
To be able to use Rviz or Gazebo on WSL that will communicate with rosmaster node on the UAV, type:
```
export ROS_MASTER_URI=http://<ip of the vehicle>:11311
```
in your WSL terminal every time you start it. </br>

You can add this line to your .bashrc file for convenience.

### ~/.bashrc
Make sure you have all of these lines added to the end of your ~/.bashrc file:
```
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2; exit;}'):0.0
export LIBGL_ALWAYS_INDIRECT=
/etc/init.d/dbus start &> /dev/null
source /opt/ros/melodic/setup.bash
source <path to catkin_ws folder>/devel/setup.bash
export ROS_MASTER_URI=http://<ip of the vehicle>:11311
export GCC4MBED_DIR=~/gcc4mbed
export ROS_LIB_DIR=~/ros/lib/ros_lib
```

## VS Code setup
Instal Remote - SSH extension.

Previous: [ROS](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/03_ros.md)</br>
Next: [Real time hardware](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/05_rt_hardware_preparation.md)
