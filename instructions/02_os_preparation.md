# OS preparation
## Jetson Nano OS preparation
Install Jetson Nano OS as described [here](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit). </br></br>
Reinstall libcanberra-gtk-module:
```
sudo apt-get install --reinstall libcanberra-gtk-module
```
## Configure wifi
Find name of your wifi device:
```
ifconfig
```
Enable it:
```
ifconfig <your wifi card/dongle>
```
Add it to interfaces:
```
sudo nano /etc/network/interfaces
```
And insert this:
```
auto <your wifi card/dongle>
iface <your wifi card/dongle> inet dhcp
  wpa-ssid <ssid>
  wpa-psk <password>
```
Save file.

Connect to wifi:
```
sudo dhclient <your wifi card/dongle>
```
Now, you should be able to connect to the UAV and control it without wired connection.

## Configure access point
TODO
## Configure serial devices
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

## Finishing up
### Reboot machine 
```
sudo reboot
```
Previous: []() </br>
Next: []()