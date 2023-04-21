# Scout-greenhouse-navigation
This repository contains ROS (Robot operating system) source files for controlling and navigating AgileX-SCOUT MINI mobile robot optimised for moving through narrow corridors of a greenhouse, for automatic gardening purposes.
## Scout MINI setup
In our case Scout mobile robot was equipped with a Sick Nav350 2D lidar sensor for localization and mapping, custom made lifter for lifting plants in a greenhouse and an Intel NUC PC unit running Ubuntu 20.04.
<p align="center">
<img src= "https://user-images.githubusercontent.com/73703833/225953506-bf8014ba-863a-4a6a-a84f-9cd61af266ad.jpeg" width="400" height="400" align="center">
</p>

## Instructions for starting the robot navigation and powering on lifting mechanism
Firstly, make sure you are connected to a robot via remote monitor/PC, we recommend using NoMachine application for remote access to the Intel NUC PC.
Once you are connected to a Scout robot, open new tmux session and run commands in following order (each in separate tmux pane):


### Starting robot drives and navigation stack

1. <code>source ~/YOUR_WORKSPACE_NAME/devel/setup.zsh</code>  -> this sources catkin workspace
2. <code>roscore</code> -> starts ROS master node
3. <code>rosrun scout_bringup bringup_can2usb.bash</code> -> launch and configuration files to start ROS nodes for controlling robot
4. <code>roslaunch scout_base scout_mini_omni_base.launch</code> -> ROS wrapper around ugv_sdk to monitor and control the scout robot
5. <code>roslaunch scout_description description.launch model_xacro:=/PATH_TO_YOUR_WORKSPACE/scout_description/urdf/scout_mini.xacro </code> -> URDF model for the mobile base
6. <code>roslaunch sicktoolbox_wrapper sicknav350.launch</code> -> driver za Sick lidar
7. <code>rosrun sicktoolbox_wrapper filter_scan.py</code> -> node that filters scans from the back of the robot in order to ignore lifting construction located on the robot
8. <code>rosrun cartographer_ros cartographer_node -configuration_directory ~/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files/ -configuration_basename scout-cartographer.lua scan:=scan_filtered</code> -> takes laser scans, creates submaps and forwards them to the cartographer_occupancy_grid_node
9. <code>rosrun cartographer_ros cartographer_occupancy_grid_node</code> -> takes cartographer submaps and returns occupancy grid which then navigation stack uses
10. <code>roslaunch scout_ros_nav navigation_cartographer.launch</code> -> launches navigation stack
11. <code>rviz</code> -> launches rviz, for visualization purposes

Now that everything is up and running, drive the robot around the space in order to create map which can be seen in rviz window. Also, in rviz you can set goal position and robot will autonomously follow produced path and reach the given goal.

### Lifting mechanism
In order to control lifting mechanism on the robot run following command that enables serial communication with arduino board controlling compressor: 
<code>putty /dev/ttyUSB0 -serial -sercfg 9600,8,n,1,N</code> \
New window will pop up, now press 1 to lift up construction and 0 to release.

## Tuning robot's navigation stack
In order to obtain reliable movement of the robot through the narrow corridors of the greenhouse, we had to adjust the parameters of the navigation stack. Robot's movement needs to be safe and reliable, and produced path needs to go through a middle of narrow corridors.

Parameters that we have tuned are located in `src\scout_navigation\scout_ros_nav\param` folder, specifically we have tuned parameters in `planner.yaml`, `costmap_local.yaml`, `costmap_global_static.yaml` and `costmap_common.yaml` yaml files.

In tuning process we have used [ROS Navigation Tuning Guide](https://kaiyuzheng.me/documents/navguide.pdf) as the main reference for setting parameters, help of other colleagues that already had experience with a scout robot and good old trial and error method. \
Some of the parameters that gave biggest improvement in terms of robot's navigation and movement through narrow corridors are listed below:

- in `planner.yaml` under **DWAPlannerROS**: 
  - `holonomic_robot: true` (because robot has omni-directional wheels)
  - `sim_time: 1.5` 
  - `path_distance_bias: 25`
  - `goal_distance_bias: 20`
  - `occdist_scale: 100`
 
- in `costmap_local.yaml`:
  - `inflation_radius: 0.13`

- in `costmap_global_static.yaml`:
  - `inflation_radius: 0.5`

- in `costmap_common.yaml`:
  - `footprint: [ [-0.2,-0.15], [0.2,-0.15], [0.2,0.15], [-0.2,0.15] ]`
  - `resolution: 0.025`
  - `obstacle_range: 6.0`
  - `raytrace_range: 6.0`

These are only few selected ones, rest of the parameters can be found in aforementioned folder.

Also, the <code>filter_scan.py</code> node was created, located in `src\sicktoolbox_wrapper\ros\sicknav350` folder, that subscribes to the `/scan topic`, removes scans approx. 30° on the each side from the back of the robot and publishes new "filtered" scans on the `/scan_filtered` topic. This needs to be done because of a lifting construction legs that are located on the back of the robot and get in the way of laser scans.
