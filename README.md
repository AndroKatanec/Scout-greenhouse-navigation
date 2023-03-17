# Scout-smart-garden
This repository contains ROS (Robot operating system) source files for controlling and navigating AgileX-SCOUT MINI mobile robot optimised for moving through narrow corridors of a greenhouse, for automatic gardening purposes.
## Scout MINI setup
In our case Scout mobile robot was equipped with a Sick Nav350 2D lidar sensor for localization and mapping, custom made lifter for lifting plants in a greenhouse and an Intel NUC PC unit running Ubuntu 20.04.
<p align="center">
<img src= "https://user-images.githubusercontent.com/73703833/225953506-bf8014ba-863a-4a6a-a84f-9cd61af266ad.jpeg" width="400" height="400" align="center">
</p>

## Instructions for starting the robot navigation and powering on lifting mechanism
Firstly, make sure you are connected to a robot via remote monitor/PC, we recommend using NoMachine application for remote access to the Intel NUC PC.
Once you are connected to a Scout robot, open new tmux session and start comands/nodes in following order (each in separate tmux windows):

1. roscore
2. 
