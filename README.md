# tracking_camera
This package is designed to get odometry data in the form of "nav_msgs/Odometry" from 
Intelrealsense tracking camera (T265) visual odometry sensor with ROS. Right now I have tested it only on Jetson Xavier NX board.

### The parameters assumed in the code :
* Samples to calculate covariances = 2000
* ROS distribution = Noetic
* frame_id = 'camera_odom_frame'
* child_frame_id = 'base_link'

### Instructions to use:
* This package needs librealsense2 package to build a node, so make sure you have it. If you don't have it, install it using following command
```
sudo apt install ros-noetic-librealsense2
```
* If your run this on ubuntu desktop then it might throw some errors while building the node, that is because I have developed it on Jetson Xavier NX
 and not tested it on ununtu desktop
* Launch tracking_camera_t265.launch file
```
roslaunch tracking_camera tracking_camera_t265.launch
```
* Keep tracking camera stationary until you see a ROS log 'Publishing on topic camera/odom' in the terminal
