# Scout-smart-garden
This repository contains ROS (Robot operating system) source files for controlling and navigating AgileX-SCOUT MINI mobile robot optimised for moving through narrow corridors of a greenhouse, for automatic gardening purposes.
## Scout MINI setup
In our case Scout mobile robot was equipped with a Sick Nav350 2D lidar sensor for localization and mapping, custom made lifter for lifting plants in a greenhouse and an Intel NUC PC unit running Ubuntu 20.04.
<p align="center">
<img src= "https://user-images.githubusercontent.com/73703833/225953506-bf8014ba-863a-4a6a-a84f-9cd61af266ad.jpeg" width="400" height="400" align="center">
</p>

## Instructions for starting the robot navigation and powering on lifting mechanism
Firstly, make sure you are connected to a robot via remote monitor/PC, we recommend using NoMachine application for remote access to the Intel NUC PC.
Once you are connected to a Scout robot, open new tmux session and run commands in following order (each in separate tmux windows):


### Starting all robot drives and navigation stack
1. source ~/YOUR_WORKSPACE_NAME/devel/setup.zsh (sources catkin workspace)
2. roscore
3. rosrun scout_bringup bringup_can2usb.bash
4. roslaunch scout_description description.launch model_xacro:=/home/larics/scout_ws/src/scout_ros/scout_description/urdf/scout_mini.xacro
5. roslaunch sicktoolbox_wrapper sicknav350.launch
6. rosrun sicktoolbox_wrapper filter_scan.py
7. rosrun cartographer_ros cartographer_node
8. rosrun cartographer_ros cartographer_occupancy_grid_node
9. roslaunch scout_ros_nav navigation_cartographer.launch
10. rviz

### Starting lifting mechanism
1. putty /dev/ttyUSB0 -serial -sercfg 9600,8,n,1,N
2. press 1 to lift up, and 0 to release
