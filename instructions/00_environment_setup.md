# Environment setup

## Setup WSL 2
### Setup Ubuntu subsystem for windows
https://docs.microsoft.com/en-us/windows/wsl/install-win10

### Then setup GUI for WSL 2
https://medium.com/@japheth.yates/the-complete-wsl2-gui-setup-2582828f4577

**Note:** Always start VcXsrv with -nowgl parameter.

### Establish connection with UAV
To be able to use Rviz or Gazebo on WSL that will communicate with rosmaster node on the UAV, type:
```
export ROS_MASTER_URI=http://<ip of the vehicle>:11311
```
in your WSL terminal every time you start it. </br>

You can add this line to your .bashrc file after "OS preparation" step has been completed.

Next: []()
