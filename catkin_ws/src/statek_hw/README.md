# statek_hw
Package with all hardware related stuff

## Launch files
### statek.launch
Launches all hardware stuff along with it's transforms.

### real_time.launch
Launches real time related stuff.

### twist_to_differential.launch
Launches bridge between geometry_msgs/Twist and statek_msgs/Velocity

## Nodes
### real_time_param_sender_node.py
Sends config to real time hardware once.

### twist_to_differential_node.py
See twist_to_differential.launch

### vision_publisher_node
Publishes state of both cameras using image_transport and advertises service to set camera info.

### vision_publisher_node_DEPRECATED.py
Use vision_publisher_node instead.
