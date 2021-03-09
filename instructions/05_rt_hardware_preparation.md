# Real time hardware
All steps described here should be performed on **WSL 2** and **not** on Jetson Nano unless stated otherwise.
## Setup ROS
Setup ROS again as described in [04_ros.md](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/04_ros.md) ._. (skip cv_bridge part)

On the bright side, you will be able to use project's launch files such as Rviz visualization or
teleop nodes on your PC. To do so, set ROS_MASTER_URI
environmental variable as described in [00_environment_setup.md](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/00_environment_setup.md).
This will allow to perform supervisory control over UAV.

## Install gcc4mbed
As described in [here](https://github.com/adamgreen/gcc4mbed).

## Compile and upload rt code
Find source code:
```
roscd statek_rt/rt_src
```

Build it:
```
make
```

**On Jetson Nano:** find connected Nucleo board (board with 512 bytes and 1 sector):
```
sudo fdisk -l
```

Assuming it's /dev/sda, mount it:
```
udisksctl mount -b /dev/sda
```

**Again, on WLS 2:** Upload compiled code:
```
scp NUCLEO_F103RB/rt.bin <UAV's username>@<UAV's ip>:/media/statek/NODE_F103RB
```

Previous: [Remote environment setup](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/04_remote_environment_setup.md)</br>
Next: [Sensor calibration](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/06_sensor_calibration.md)
