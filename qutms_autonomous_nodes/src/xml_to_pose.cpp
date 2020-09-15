// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>

// C++ includes
#include <regex>
#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>

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
    ros::Publisher pose_pub = np.advertise<nav_msgs::Path>("/qev/path_poses", 10, true);
    
    // Rate set
    ros::Rate rate(5); // Set 5Hz 
    
    // Open a file
    sprintf(dir, "/home/nick-pc-ubuntu/catkin_ws/src/qutms_autonomous_nodes/pose_files/skidpad_track/track.txt");
    
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
            if (regex_search(got, regex("<pose>"))) {
                // Notify we have found a match
                // ROS_INFO("Match found!");
                //cout << got << endl;

                // Define some words to find and remove 
                string wrd_1 = "</pose>";
                string wrd_2 = "<pose>";
                size_t pos_1 = got.find(wrd_1);
                size_t pos_2 = got.find(wrd_2);

                // Remove each word
                if((pos_1 != string::npos) && (pos_2 != string::npos)) {
                    got.erase(pos_1, wrd_1.length());
                    // cout << got << endl;
                    got.erase(pos_2, wrd_2.length());
                    // cout << got << endl;
                }
                // Split the string
                vector<string> str_res;
                boost::split(str_res, got, boost::is_any_of(" "));
                // cout << "Result string is " << str_res.size() << " long" << endl;
                // Check result
                vector<double> tmp_vec;
                for(int i = 12; i < str_res.size(); i++) {
                    try {
                        // Try to cast the string to a number, and store it
                        // cout << "Trying to cast " << str_res[i] << " to a number" << endl;
                        double tmp;
                        tmp = boost::lexical_cast<double>(str_res[i]);
                        tmp_vec.push_back(tmp);
                        // ROS_INFO("Successfully stored number %2.4f", tmp);
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
                //ROS_INFO("Adding numbers %2.4f, %2.4f, %2.4f to vector containers", tmp_vec[0], tmp_vec[1], tmp_vec[2]);

            }
            }
        }

    // Close up
    infile.close();
    ROS_INFO("File closed");

    // Do some setup
    ROS_INFO("Setting up path...");
    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped path_pose;

    // Split everything up into blue and yellow poses
    vector<double> xb, yb, xy, yy;
    if (xg.size() == yg.size()) {
        for (int i = 0; i <= 22; i++) {
            xb.push_back(xg[0]);
            xg.erase(xg.begin());
            yb.push_back(yg[0]);
            yg.erase(yg.begin());

            xy.push_back(xg[23-i]);
            xg.erase(xg.begin()+(23-i));
            yy.push_back(yg[23-i]);
            yg.erase(yg.begin()+(23-i));
        }
    }

    // Sort the lists
    vector<double> xyd, yyd;
    vector<double>::iterator cindex;
    int cpos;
    while((!xy.empty()) && (!yy.empty())) {
        // Euclidean distances
        vector<double> cdists;
        for(int ii = 0; ii < xb.size(); ii++) {
            cdists.clear();
            for(int jj = 0; jj < xy.size(); jj++) {
                double cdist = sqrt(((xb[ii] - xy[jj])*(xb[ii] - xy[jj])) + ((yb[ii] - yy[jj])*(yb[ii] - yy[jj])));
                cdists.push_back(cdist);
                // ROS_INFO("Calculated distance as %0.4f", cdist);
            }
        }

        // Find the minimum
        auto min = *min_element(cdists.begin(), cdists.end());
        cindex = find(cdists.begin(), cdists.end(), min);
        cpos = distance(cdists.begin(), cindex);
        ROS_INFO("Found distance at element %d", cpos);

        // Push these values into the path, and remove those values
        xyd.push_back(xy[cpos]);
        yyd.push_back(yy[cpos]);

        xy.erase(xy.begin() + cpos);
        yy.erase(yy.begin() + cpos);        
    }

    ROS_INFO("Getting midpoints");
    vector<double> xp, yp;
    if(xb.size() == xyd.size()) {
        for(int ii = 0; ii < xb.size(); ii++) {
            // Store the points
            double xm = (xb[ii] + xy[ii])/2;
            // ROS_INFO("Calculated x midpoint as: %2.4f", xm);
            xp.push_back(xm);
        }
    } else {
        ROS_ERROR("Lists of x coordinates are not the same size");
    }
    
    if(yb.size() == yyd.size()) {
        for(int ii = 0; ii < yb.size(); ii++) {
            // Store the points
            double ym = (yb[ii] + yy[ii])/2;
            // ROS_INFO("Calculated y midpoint as: %2.4f", ym);
            yp.push_back(ym);
        }
    } else {
        ROS_ERROR("Lists of y coordinates are not the same size");
    }

    // Enter the loop 
    int seq = 0;

    // Add a pose
    ROS_INFO("Adding poses to Path message");
    // Add the first pose
    path_pose.pose.position.x = xp[0];
    path_pose.pose.position.y = yp[0];
    path_pose.pose.position.z = 0;
    path_pose.pose.orientation.w = 0;
    path_pose.pose.orientation.x = 0;
    path_pose.pose.orientation.y = 0;
    path_pose.pose.orientation.z = 0;        
    path_msg.poses.push_back(path_pose);

    // Remove it from the vector
    xp.erase(xp.begin());
    yp.erase(yp.begin());
    vector<double>::iterator index;
    int pos;
    
    // Get distances from last path point to all other points, get minimum distance, add corresponding elements and remove
    while((!xp.empty()) && (!yp.empty())) {
        // Euclidean distances
        vector<double> dists;
        for(int ii = 0; ii < xp.size(); ii++) {
            double dist = sqrt(((path_pose.pose.position.x - xp[ii])*(path_pose.pose.position.x - xp[ii])) + ((path_pose.pose.position.y - yp[ii])*(path_pose.pose.position.y - yp[ii])));
            dists.push_back(dist);
            ROS_INFO("Calculated distance for elements %d to be %0.4f", ii, dist);
        }

        // Find the minimum
        auto min = *min_element(dists.begin(), dists.end());
        index = find(dists.begin(), dists.end(), min);
        pos = distance(dists.begin(), index);
        ROS_INFO("Found distance at element %d", pos);

        // Push these values into the path, and remove those values
        path_pose.pose.position.x = xp[pos];
        path_pose.pose.position.y = yp[pos];
        path_pose.pose.position.z = 0;
        path_pose.pose.orientation.w = 0;
        path_pose.pose.orientation.x = 0;
        path_pose.pose.orientation.y = 0;
        path_pose.pose.orientation.z = 0;        
        path_msg.poses.push_back(path_pose);

        xp.erase(xp.begin() + pos);
        yp.erase(yp.begin() + pos);        
    }

    ROS_INFO("Entering main loop");
    while(ros::ok()) {
        // Add everything else to the message
        path_msg.header.seq = seq;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        // Publish
        pose_pub.publish(path_msg);
        seq++;
        rate.sleep();
        
    }

    return 0;
}