# Sensor calibration
## Camera calibration
Print ![Chessboard](chessboard.png) and measure width of one black square

Run:
```
rosdep install camera_calibration
rosmake camera_calibration
```

Then run:
```
roslaunch statek_calibrate camera_info_calibrate.launch size:=<SIZE OF BLACK SQUARE IN METERS>
```
Follow [camera_calibration](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) from point 4 and after calibration click COMMIT button.
If the result is satisfying then kill launch file and proceed to next step.
