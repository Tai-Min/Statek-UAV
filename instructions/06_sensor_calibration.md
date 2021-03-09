# Sensor calibration
## Camera calibration
Print </br> 
![Chessboard](chessboard.png) </br> 
and measure width of one black square.

On WSL 2 run once:
```
rosdep install camera_calibration
rosmake camera_calibration
```
Then every time you want to calibrate the camera run:
```
roslaunch statek_calibrate camera_info_calibrator.launch square_size:=<SIZE OF BLACK SQUARE IN METERS>
```
Follow [camera_calibration](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) from point 4.
If the result is satisfying then click COMMIT button and kill launch file and proceed to next step. If camera_calibration node returned some errors / can't communicate with vision node then you can copy content of **D, K, P, R** matrices into **statek_config/yaml/camera_info_left.yaml** and **statek_config/yaml_camera_info_right.yaml**

## Depth calibration
On WSL 2 run:
```
roslaunch statek_calibrate depth_calibrator.launch
```
In rqt reconfigure window navigate to statek/stereo/stereo_image_proc and tune parameters so disparity window and point cloud in Rviz looks good. Save parameters using rqt reconfigure gui into statek_calibrate package into /yaml/ folder as depth_config.yaml

Previous: []() </br>
Next: []()
