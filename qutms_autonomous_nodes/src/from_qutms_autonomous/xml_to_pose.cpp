// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>

// C++ includes
#include <regex>
#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

char dir[300];
string got;


vector<double> get_points(vector<double> x_points, vector<double> y_points, int itr) {
    // Sorts 4 points 

    vector<double> points;
    // Sanity check
    if(x_points.size() != y_points.size()) {
        ROS_ERROR("Vectors aren't the same size!");
    }

    // If we are close to the end of the vector, wrap the iterator
    if (itr == x_points.size() - 2) { // If we are 2 away from the end 
        points.push_back(x_points[itr]);
        points.push_back(y_points[itr]);
        points.push_back(x_points[itr + 1]);
        points.push_back(y_points[itr + 1]);
        points.push_back(x_points[itr + 2]);
        points.push_back(y_points[itr + 2]);
        points.push_back(x_points[0]); // Go back to the beginning
        points.push_back(y_points[0]);

    } else if (itr == x_points.size() - 1) { // If we are one away
        points.push_back(x_points[itr]);
        points.push_back(y_points[itr]);
        points.push_back(x_points[itr + 1]);
        points.push_back(y_points[itr + 1]);
        points.push_back(x_points[0]);
        points.push_back(y_points[0]);
        points.push_back(x_points[1]);
        points.push_back(y_points[1]);
    } else if (itr == x_points.size()) { // If we have reached the maximum size
        points.push_back(x_points[itr]);
        points.push_back(y_points[itr]);
        points.push_back(x_points[0]);
        points.push_back(y_points[0]);
        points.push_back(x_points[1]);
        points.push_back(y_points[1]);
        points.push_back(x_points[2]);
        points.push_back(y_points[2]);
    } else {
        for(int i = 0; i <= 3; i++) { // All other cases
            points.push_back(x_points[itr + i]);
            points.push_back(y_points[itr + i]);
        }
    }
    return points;
    
}

int main(int argc, char** argv) {
    // Initilize node
    ros::init(argc, argv, "xml_pose_grabber");

    // Node Handle
    ros::NodeHandle np;

    // Publisher goes here (latch it)
    ros::Publisher pose_pub = np.advertise<geometry_msgs::PoseArray>("/qev2/landmark_poses", 10);
    
    // Rate set
    ros::Rate rate(3); // Set 5Hz 
    
    // Open a file
    sprintf(dir, "/home/nick-pc-ubuntu/catkin_ws/src/qutms_autonomous/models/Lakeside_test_track/track.txt");
    
    // Set up some variables
    ifstream infile;
    vector<double> xg, yg, zg, rollg, pitchg, yawg;

    // Open the file up
    infile.open(dir);
    //cmatch cm;
    if(infile.good()) {
        // Pass the line for checking
        ROS_INFO("Checking file...");
        while(getline(infile, got)) {
            if (regex_search(got, regex("frame="))) {
                // Notify we have found a match
                ROS_INFO("Match found!");
                //cout << got << endl;

                // Define some words to find and remove 
                string wrd_1 = "</pose>";
                string wrd_2 = "<pose frame=''>";
                size_t pos_1 = got.find(wrd_1);
                size_t pos_2 = got.find(wrd_2);

                // Remove each word
                if((pos_1 != string::npos) && (pos_2 != string::npos)) {
                    got.erase(pos_1, wrd_1.length());
                    //cout << got << endl;
                    got.erase(pos_2, wrd_2.length());
                    //cout << got << endl;
                }
                // Split the string
                vector<string> str_res;
                boost::split(str_res, got, boost::is_any_of(" "));
                // Check result
                vector<double> tmp_vec;
                for(int i = 6; i < 12; i++) {
                    try {
                        // Try to cast the string to a number, and store it
                        double tmp;
                        tmp = boost::lexical_cast<double>(str_res[i]);
                        tmp_vec.push_back(tmp);
                        //ROS_INFO("Successfully stored number %0.3f", tmp);
                    } catch(boost::bad_lexical_cast &e) {
                        ROS_ERROR("Couldn't cast string to double, adding a 0");
                        tmp_vec.push_back(0);
                    }
                }
                xg.push_back(tmp_vec[0]);
                yg.push_back(tmp_vec[1]);
                zg.push_back(tmp_vec[2]);
                rollg.push_back(tmp_vec[3]);
                pitchg.push_back(tmp_vec[4]);
                yawg.push_back(tmp_vec[5]);
                //ROS_INFO("Adding numbers %0.3f, %0.3f, %0.3f to vector containers", tmp_vec[0], tmp_vec[1], tmp_vec[2]);

            }
            }
        }

    // Close up
    infile.close();
    ROS_INFO("File closed");

    // Do some setup
    ROS_INFO("Setting up arrays...");
    geometry_msgs::PoseArray pose_msg;
    geometry_msgs::Pose landmark_poses;

    // Split everything up into blue and yellow poses
    vector<double> xb, yb, xy, yy;
    if (xg.size() == yg.size()) {
        for (int i = 0; i <= 202; i++) {
            xb.push_back(xg[0]);
            xg.erase(xg.begin());
            yb.push_back(yg[0]);
            yg.erase(yg.begin());
        }

        for (int i = 0; i <= 196; i++) {
            xy.push_back(xg[0]);
            xg.erase(xg.begin());
            yy.push_back(yg[0]);
            yg.erase(yg.begin());
        }
    }
    
    // Enter the loop 
    int seq = 0;
    int ib = 0;
    int iy = 0;
    while(ros::ok()) {
        // Clear the message
        pose_msg.poses.clear();

        if (ib > xb.size()) {
            ib = 0;
        }
        if (iy > xy.size()) {
            iy = 0;
        }
        
        // Get 4 points at a time from both sets
        vector<double> bpoints, ypoints;
        bpoints = get_points(xb, yb, ib);
        ypoints = get_points(xy, yy, iy); 
        //ROS_INFO("Returned %lu blue landmarks and %lu yellow landmarks", bpoints.size()/2, ypoints.size()/2);

        // Put to the message
        for (int i = 0; i <= 3; i++) {
            landmark_poses.position.x = bpoints[2*i];
            landmark_poses.position.y = bpoints[2*i + 1];
            pose_msg.poses.push_back(landmark_poses);
            landmark_poses.position.x = ypoints[2*i];
            landmark_poses.position.y = ypoints[2*i + 1];
            pose_msg.poses.push_back(landmark_poses);
        }

        pose_msg.header.seq = seq;
        pose_msg.header.frame_id = "track";

        // Publish
        pose_pub.publish(pose_msg);
        seq++;
        ib++;
        iy++;
        rate.sleep();
        
    }

    return 0;
}