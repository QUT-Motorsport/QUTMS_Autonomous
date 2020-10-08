# QUTMS autonomous system - WIP
This repo aims the explain and document our development of autonomus driverless system. 

__WORK IN PROGRESS__ 

## How to use
From the new_package branch grab the qutms_autonomous folder. Place this into your catkin_ws/src directory.
Run these commands to install dependencies
```
sudo apt install python-catkin-tools
sudo apt install ros-melodic-hector-gazebo-plugins
sudo apt install ros-melodic-teleop-twist-keyboard
sudo apt install ackermann-msgs
sudo apt install ros-melodic-joy
```

## Setting up your workspace
Run the following in your catkin_ws to clean your devel and build directories
```
catkin clean --yes
```
If all goes well then run
```
catkin build
```
Use catkin build to make executables from now on
