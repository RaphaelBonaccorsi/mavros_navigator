# Mavros Navigator
ROS 2 package for navigating using MavROS, it has a action server that receives PoseStamped waypoints and go to them.

## Architecture

![mavros_navigator_architecture](https://github.com/user-attachments/assets/213e5f83-22da-4fbd-9d50-acf7ad9ef12e)

## System requirements

This package was tested in an Ubuntu 22.04 environment using ROS Humble

## Instalation

### ROS 2
Tested with ROS Humble
https://docs.ros.org/en/humble/Installation.html

### PX4 Autopilot ([see this link](https://docs.px4.io/main/en/simulation/) )
- git clone https://github.com/PX4/PX4-Autopilot.git --recursive
- bash PX4_Autopilot/Tools/setup/ubuntu.sh
#### Use gazebo classic instead of Ignition Gazebo
- sudo apt remove gz-harmonic
- sudo apt install aptitude
- sudo aptitude install gazebo libgazebo11 libgazebo-dev
- cd PX4_Autopilot
- make px4_sitl gazebo-classic

### Mavros
- sudo apt install ros-humble-mavros
- source /opt/ros/humble/setup.basah
- ros2 run mavros install_geographiclib_datasets.sh

## Building
- source /opt/ros/{ROS_DISTRO}/setup.bash
- On the package folder
- colcon build

## Running the example

### Terminal 1
- cd PX4-Autopilot/
- export PX4_HOME_LAT={HOME_LATITUDE}; export PX4_HOME_LON={HOME_LONGITUDE}; export PX4_HOME_ALT={HOME_ALTITUDE}; make px4_sitl gazebo-classic HEADLESS=1
- The headless option is to disable the gazebo GUI.
- "gazebo-classic" part may be different accordingly to your simulation setup, [see this for more information](https://docs.px4.io/main/en/simulation/)
  
### Terminal 2
- source /opt/ros/{ROS_DISTRO}/setup.bash
- ros2 run mavros mavros_node --ros-args --param fcu_url:=udp://:14540@
  
### Terminal 3
- source /opt/ros/{ROS_DISTRO}/setup.bash
- On the package folder
- source install/setup.bash
- ros2 launch launch/example_launch.py

### About the example
- The "path_planner.py" node is just a simple example that reads x and y files, this is a very simple implementation designed specifically for a purpose in another system, you should change it for your application
