# Real time hardware

## Install necessary libraries
In PlatformIO terminal type:
```
pio lib -g install https://github.com/Tai-Min/MPU9250/archive/refs/heads/no-verbose.zip
```

## Compile and upload rt code
Connect STM board to PC.

Find source code:
```
roscd statek_hw/rt_src
```
Open it on Windows in VS code with PLATFORMIO plugin, and compile then upload the code.

Previous: [Remote environment setup](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/04_remote_environment_setup.md)</br>
Next: [Sensor calibration](https://github.com/Tai-Min/Statek-UAV/blob/master/instructions/06_sensor_calibration.md)
