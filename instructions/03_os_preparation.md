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

## Access point preparation

## Finishing up
### Reboot machine 
```
sudo reboot
```
Previous: []() </br>
Next: []()