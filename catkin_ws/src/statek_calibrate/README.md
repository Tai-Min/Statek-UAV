# statek_calibrate
## Launch files
### camera_info_calibrator.launch
Launches calibration GUI so should be run on remote machine.
The result of this node is automatically saved.

### depth_calibrator.launch
Launches dynamic reconfigure along with stereo preview and rviz with point cloud. 
Resulting params should be saved manually in dynamic reconfigure as depth_config.yaml and moved to UAV to statek_config/yaml

### motion_calibrator.launch
Launches simple terminal app to calibrate motion this includes:
* Physical constraints
* Motor PID tuning which includes:
  * Step response data acquisition
  * n-th order discrete model identification using recursive least squares
  * Least squares minimalization using cost function that is expressed as result of (ISE + IAE)/2 + penalty from overshoot and CV saturation
* IMU calibration

## Nodes
### motion_calibration_node.py
See motion_calibrator.launch
