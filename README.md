# QUTMS autonomous system - WIP
This repo aims the explain and document our development of autonomus driverless system. 

__WORK IN PROGRESS__ 

## How to use
From the new_package branch grab the two folders qutms_autonomous and velodyne_gazebo_plugins. Place these into your catkin_ws/src directory.
Run these commands to install dependencies
```
sudo apt install python-catkin-tools
sudo apt install ros-melodic-hector-gazebo-plugins
sudo apt install ros-melodic-teleop-twist-keyboard
```

## Setting up your workspace
Run the following in your catkin_ws to clean your devel and build directories
```
catkin clean
```
If all goes well then run
```
catkin build
```
