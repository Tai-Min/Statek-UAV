# Setup workspace

## Setup WSL 2
Setup Ubuntu subsystem for windows:
https://docs.microsoft.com/en-us/windows/wsl/install-win10

Then setup GUI for it:
https://medium.com/@japheth.yates/the-complete-wsl2-gui-setup-2582828f4577

Start VcXsrv with -nowgl parameter.

Then, to connect to ROS stuff, in your WSL terminal type:
```
export ROS_MASTER_URI=http://<ip of the vehicle>:11311
```

## Configure wifi access
Find name of your wifi device:
```
ifconfig
```
Enable it:
```
ifconfig <your wifi card/dongle>
```
Connect:
```
iwconfig <your wifi card/dongle> essid <ssid> key <password>
dhclient <your wifi card/dongle>
```


Next: []()
