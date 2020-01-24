# QUT autonomous

## How to use
### Catkin workspace
Start by creating a catkin workspace:   
```mkdir -p ~/catkin_ws/src```   
```cd ~/catkin_ws/```   
Here, to initialise from scratch do as official ROSwiki suggests:    
```catkin_make```    

Or just copy and replace with this repo's subdirectory `catkin_ws-DO_NOT_USE/`.
_Notes: there will likely be pathing errors. Fix accordingly in CMakeCache.txt and Makefile by replacing the correct absolute path._ 

The reason why the catkin_ws is appended with DO_NOT_USE, is because this is not an appropriate setup. 
The setup of a catkin_ws should follow ROSwiki's explicit instructions.    

One may require to do `catkin_make` at `catkin_ws` directory anyway.

### Using Gazebo
Given a correct ROS installation, there should be a `.gazebo/` folder under root, i.e.:   
```~/.gazebo/```

In which, there should be a `models/` folder that we can place our models in by doing:   
```mkdir ~/.gazebo/models/QEV2```   
_Notes: the folder name must be 'QEV2' to align with ROS system_


### Executing 
Typically, the `.world` can be run by simply doing:    
```gazebo _file_.world```

However, we eventually will end up using `roslaunch` and other ROS related commands. And that's what this README is trying to explain. 

Given the Catkin workspace is set up correctly, and if you haven't already, do:   
```cd ~/catkin_ws```   
```source devel/setup.bash```   

This should allow ROS filesystem to work. 
So now we do:  
```roslaunch qutms_autonomous_gazebo test.launch```

This should launch gazebo with a ground plane and our QEV2 model.





































