// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// C++ includes
#include <math.h>
#include <iostream>
#include <fstream>

// Used only for skidpad; creates a skidpad path given certain parameters
// Inputs: track radius, cone separation, point resolution (number of points per loop)
// Note: text file has 23 blues, 23 yellows and 

char dirt[300];

using namespace std;

class Skidpad_Mapper {

    public: 
    Skidpad_Mapper();

    void create_map(void);
    void make_points(void);

    private:
    ros::NodeHandle np;
    ros::Publisher path_publisher;
    geometry_msgs::PoseStamped point;
    nav_msgs::Path track_path;

    // Parameters
    double track_radius_inner, track_radius_outer, cone_res, track_enter, track_exit, track_mid, track_circle_sep;
    vector<double> x_points, y_points, f_points;
};

// Constructor
Skidpad_Mapper::Skidpad_Mapper() {
    // Publisher (latched)
    path_publisher = np.advertise<nav_msgs::Path>("/qev/skidpad_map", 10, true);

    // Set variables here

}

void Skidpad_Mapper::create_map(void)  {
    // Use this function to make a map for the skidpad track

    // Get the file path
    sprintf(dirt, "/home/nick-pc-ubuntu/catkin_ws/src/qutms_autonomous_nodes/pose_files/skidpad_track/track.txt");

    // Variables for processing text file
    string line;
    ifstream tinfile (dirt);

    // Make sure the vectors are empty
    x_points.clear();
    y_points.clear();

    // Open and get the data
    if(tinfile.is_open()) {
        // Whilst we have lines to process, process them
        while(getline(tinfile, line)) {
            double xtxt, ytxt;
            tinfile >> xtxt, ytxt;
            // cout << "Got " << xtxt << " " << ytxt << endl;
            // Save to vectors
            x_points.push_back(xtxt);
            y_points.push_back(ytxt);
        }
    } else {
        ROS_ERROR("Could not open text file!");
    }

    // Close the file
    tinfile.close();

}

void Skidpad_Mapper::make_points(void) {
    // Generates all midpoints 
    double pnt = 
}

