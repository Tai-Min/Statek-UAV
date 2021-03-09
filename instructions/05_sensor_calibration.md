# Sensor calibration
## Camera calibration
Print </br> 
![Chessboard](chessboard.png) </br> 
and measure width of one black square

On WSL run:
```
rosdep install camera_calibration
rosmake camera_calibration
```

Then run:
```
roslaunch statek_calibrate camera_info_calibrator.launch square_size:=<SIZE OF BLACK SQUARE IN METERS> machine_address:=<UAV's IP>
```
Follow [camera_calibration](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) from point 4.
If the result is satisfying then click COMMIT button and kill launch file and proceed to next step. If camera_calibration node returned some errors / can't communicate with vision node then you can copy content of **D, K, P, R** matrices into **statek_config/yaml/camera_info_left.yaml** and **statek_config/yaml_camera_info_right.yaml**

Previous: []() </br>
Next: []()
