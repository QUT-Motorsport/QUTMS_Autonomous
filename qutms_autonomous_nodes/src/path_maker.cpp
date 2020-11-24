// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// C++ includes
#include <math.h>
#include <iostream>

// Used only for skidpad; creates a skidpad path given certain parameters
// Inputs: track radius, cone separation, point resolution (number of points per loop)

class Skidpad_Mapper {

    public: 
    Skidpad_Mapper();

    private:
    ros::NodeHandle np;
    ros::Publisher path_publisher;
    geometry_msgs::PoseStamped point;
    nav_msgs::Path track_path;

    // Parameters
    double track_radius_inner, track_radius_outer, cone_res, track_enter, track_exit, track_mid, track_circle_sep;
};

// Constructor
Skidpad_Mapper::Skidpad_Mapper() {
    // Publisher (latched)
    path_publisher = np.advertise<nav_msgs::Path>("/qev/skidpad_map", 10, true);

}

