[Previous step - Third party packages](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/thirdparty_packages.md)

# OS preparation

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
