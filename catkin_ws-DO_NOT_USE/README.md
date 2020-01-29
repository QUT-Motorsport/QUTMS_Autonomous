# This is a README / notes documenting the development process

## Catkin workspace
### WARNING
The reason why the catkin_ws is appended with DO_NOT_USE, is because this is not an appropriate setup. 
The setup of a catkin_ws should follow ROSwiki's explicit instructions. 

This workspace however, is an example to how your catkin workspace should look like.
_Notes: it's almost guaranteed to have pathing errors this way. Fix them accordingly in CMakeCache.txt and Makefile by replacing the correct absolute path._ 

### Setting up catkin workspace
Start by creating a catkin workspace: ```mkdir -p ~/catkin_ws/src```    
then, ```cd ~/catkin_ws/```   

Here, to initialise from scratch do as official ROSwiki suggests:    
```catkin_make```    

Or just copy and replace with this repo's subdirectory `catkin_ws-DO_NOT_USE/`.    
One may require to do `catkin_make` at `catkin_ws` directory anyway (after fixing all the pathing errors).

### Plugins
In `catkin_ws/src/qutms_autonomous_gazebo/plugins` contains plugin source and build files.

To build from source again, do `cd ~/catkin_ws/src/qutms_autonomous_gazebo/plugins/build`    
Then, do `cmake ../` which builds the parent directory into this build folder.   
This action generates CMakeCache, shared object, other files and a Makefile.   
Run `make`.

This should complete the plugin compilation.

_Notes:_
During dev, the custom plugins (/QEV2/vel_cmd and /QEV2/turn_cmd) were not showing up in `rostopic list`.    
The problem was later found that a explicit path (absolute or relative) must be declared in our `.world` file, in the line where we 'attached' the plugin.   
For example, our plugin.cc file is in `qutms_autonomous_gazebo/plugins`.
In which contains plugin source file as well as its cmake build folder. 
The generated .so file will appear in this build folder. 
Hence our world file, when specifying the .so file, must at least have `build/xxx.so` in front to complete a relative pathing. 

### Using Gazebo
Given a correct ROS installation, there should be a `.gazebo/` folder under root, i.e.:   
```~/.gazebo/```

In which, there should be a `models/` folder that we can place our models in by doing:   
```mkdir ~/.gazebo/models/QEV2```   
_Notes: the folder name must be 'QEV2' to align with ROS system_

Place our __`model.sdf`__ and __`model.config`__ within this folder. 

### Launching
Typically, the `.world` can be run by simply doing:    
```gazebo _file_.world```

However, we eventually will end up using `roslaunch` and other ROS related commands. And that's what this README is mostly about - launching using roslaunch. 

Given the Catkin workspace is set up correctly, and if you haven't already, do:   
```cd ~/catkin_ws```   
```source devel/setup.bash```   

This should allow ROS filesystem to work. 
So now we do:  
```roslaunch qutms_autonomous_gazebo test.launch```

This should launch gazebo with a ground plane and our QEV2 model.

### Running
By running roslaunch, a roscore will be automatically started and gazebo launched.

Open another terminal, check if custom plugins are working by typing `rostopic list`.    
You should see /QEV2/xxx topics.
If not, the plugins have not loaded properly.

If loaded, you can publish to these nodes to control the robot.
For example:     
```rostopic pub /QEV2/vel_cmd std_msgs/Float32 "data: 5.0"```

This actuates the rear wheels to move the car forward at 5.0 speed. 
Steering command (/QEV2/turn_cmd) is similar. 

































