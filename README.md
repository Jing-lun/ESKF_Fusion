# ESKF Fusion

ESKF Fusion for IMU with GNSS data and Odometry data, when GNSS data is not valid, fuse with Odometry only. The theory can be referred to [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/pdf/1711.02508.pdf). 

## Dependencies:
```
It is tested under Ubuntu 20.04 + ROS noetic.
* nav_msgs 
* Eigen
* Sophus
* GeographicLib
```

### ROS Packages
`geographic-*`, `geographiclib-*`, `libgeographic-*`: Used to convert lla to ENU coordiante

### Setup
  ````
  sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3**/Modules/ (* is the version of your cmake)
  ````

## Installation

Clone the repository to the catkin work space eg. `/catkin_ws/src`
````
git clone git@github.com:Jing-lun/ESKF_Fusion.gt
````
Compile
````
cd ~/catkin_ws
catkin_make -j
````
## Run
Turn on a terminal, and run:
````
roslaunch awv_fusion eskf.launch
````
Turn another terminal to open rosbag file that you have
