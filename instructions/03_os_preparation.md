# OS preparation
## Jetson Nano OS preparation
Install Jetson Nano OS as described [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit).

Reinstall libcanberra-gtk-module
```
sudo apt-get install --reinstall libcanberra-gtk-module
```
## Serial device preparation
Make sure that all serial devices are connected.
### Add yourself to dialout group
```
sudo usermod -a -G dialout $USER
```

### Add your lidar info to udev rules
Find info about your device (idVendor, idProduct, serial):
```
udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSBn)
```

Where /dev/ttyUSBn is the port to which your lidar is connected to.

Add new rule to udev:
```
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

Then add this line:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="<YOUR ID>", ATTRS{idProduct}=="<YOUR ID>", ATTRS{serial}=="<YOUR SERIAL>", SYMLINK+="ydlidar"
```

### Reboot machine 
```
sudo reboot
```

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

Previous: []() </br>
Next: []()