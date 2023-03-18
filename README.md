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


### Starting robot drives and navigation stack

1. <code>source ~/YOUR_WORKSPACE_NAME/devel/setup.zsh</code>  -> this sources catkin workspace
2. <code>roscore</code> -> starts ROS master node
3. <code>rosrun scout_bringup bringup_can2usb.bash</code> -> launch and configuration files to start ROS nodes for controlling robot
4. <code>roslaunch scout_base scout_mini_omni_base.launch</code> -> ROS wrapper around ugv_sdk to monitor and control the scout robot
5. <code>roslaunch scout_description description.launch model_xacro:=/PATH_TO_YOUR_WORKSPACE/scout_description/urdf/scout_mini.xacro </code> -> URDF model for the mobile base
6. <code>roslaunch sicktoolbox_wrapper sicknav350.launch</code> -> driver za Sick lidar
7. <code>rosrun sicktoolbox_wrapper filter_scan.py</code> -> node that filters scans from the back of the robot in order to ignore lifting construction located on the robot
8. <code>rosrun cartographer_ros cartographer_node -configuration_directory ~ -configuration_basename scout-cartographer.lua scan:=scan_filtered</code> -> takes laser scans, creates submaps and forwards them to the cartographer_occupancy_grid_node
9. <code>rosrun cartographer_ros cartographer_occupancy_grid_node</code> -> takes cartographer submaps and returns occupancy grid which then navigation stack uses
10. <code>roslaunch scout_ros_nav navigation_cartographer.launch</code> -> launches navigation stack
11. <code>rviz</code> -> launches rviz, for visualization purposes

### Lifting mechanism
In order to control lifting mechanism on the robot run following command: 
<code>putty /dev/ttyUSB0 -serial -sercfg 9600,8,n,1,N</code> \
New window will pop up, now press 1 to lift up construction and 0 to release.
