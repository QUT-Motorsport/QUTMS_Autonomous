// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

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
    void make_path(void);
    void path_pub(void);

    private:
    ros::NodeHandle np;
    ros::Publisher path_publisher;
    geometry_msgs::PoseStamped point;
    nav_msgs::Path track_path;

    // Parameters
    double track_radius_inner, track_radius_outer, cone_res, track_enter, track_exit, track_mid, track_circle_sep;
    vector<double> x_points, y_points, xf_points, yf_points, phi_f_points;
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
    // Sanity check: we have an equal number of points in both vectors
    if(x_points.size() != y_points.size()) {
        ROS_ERROR("Unequal number of points!");
    } else {
        // Use two points at a time to calculate midpoints
        for (int ii = 0; ii <= x_points.size(); ii = ii+2) {
            double xf = (x_points[ii] + x_points[ii+1])/2;
            double yf = (y_points[ii] + y_points[ii+1])/2;

            // Append
            xf_points.push_back(xf);
            yf_points.push_back(yf);
        }
    }

    // Get heading values
    for (int ii = 0; ii <= xf_points.size(); ii++) {
        // First heading value is 0, all others are angles between points
        if (ii == 0) {
            double phi_f = 0;

            // Append
            phi_f_points.push_back(phi_f);

        } else {
            double phi_f = atan2((yf_points[ii] - yf_points[ii-1]),(xf_points[ii] - xf_points[ii-1]));

            // Append
            phi_f_points.push_back(phi_f);
        }
    } 
}

void Skidpad_Mapper::make_path(void) {
    // Uses created points to generate the Path message to be sent 
    geometry_msgs::PoseStamped pose;

    // Make a pose then push it to the Path
    for(int ii = 0; ii <= xf_points.size(); ii++) {
        pose.pose.position.x = xf_points[ii];
        pose.pose.position.y = yf_points[ii];
        pose.pose.position.z = 0.15;

        // Convert from RPY to Quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, phi_f_points[ii]);
        pose.pose.orientation.w = quat.getW();
        pose.pose.orientation.x = quat.getX();
        pose.pose.orientation.y = quat.getY();
        pose.pose.orientation.z = quat.getZ();

        track_path.poses.push_back(pose);
    }
}

void Skidpad_Mapper::path_pub(void) {
    // Publishes the path message
    path_publisher.publish(track_path);
}

int main(int argc, char **argv) {
    // Init
    ros::init(argc, argv, "Skidpad_Map");

    // Class 
    Skidpad_Mapper skidpad_map;

    // Rate set
    ros::Rate rate(5);

    // Create the map 
    skidpad_map.create_map();

    // Create the message
    skidpad_map.make_points();
    skidpad_map.make_path();

    // Publish it 
    while(ros::ok()) {
        skidpad_map.path_pub();

        rate.sleep();
    }
    
}

