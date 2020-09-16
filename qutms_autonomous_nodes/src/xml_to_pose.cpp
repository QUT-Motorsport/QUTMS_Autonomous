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

vector<double> vector_sort(vector<double> vec_in) {
    // Sorts a vector using a nearest neighbour approach
    vector<double> vec_out;
    
    // Calculate point distances, find minimum, and append minimum to new vector
    while(!vec_in.empty()) {
        int ii = 1;
        vector<double> vdists;
        for (int a = 0; a < vec_in.size(); a++) {
            // Calculate distances
            double dist;
            dist = abs(vec_in[ii] - vec_in[0]);
            vdists.push_back(dist);
            ii++;
        }
        // Find the minimum
        auto min = *min_element(vdists.begin(), vdists.end());
        vector<double>::iterator vind = find(vdists.begin(), vdists.end(), min);
        int vpos = distance(vdists.begin(), vind);

        // Append to the new array, and delete from the old one
        vec_out.push_back(vec_in[vpos]);
        vec_in.erase(vec_in.begin() + vpos);
    }

    return vec_out;
}

void vector_dist_sort(vector<double> avec_in, vector<double> bvec_in, vector<double> cvec_in, vector<double> dvec_in, vector<double> &avec_out, vector<double> &bvec_out) {
    // Uses Euclidean distance to sort two vectors containing individual x and y coordinates with two others containing the same
    // The input arguments should be (x, y, x, y)
    // The last two arguments are vectors to be sorted
    
    // Check: vectors to be sorted are the same size
    if((avec_in.size() != cvec_in.size()) && (bvec_in.size() != dvec_in.size())) {
        ROS_ERROR("Input vectors not the same size!");
        return;
    } else {
        vector<double> vecdists;
        for (int a = 0; a < avec_in.size(); a++) {
            // Distances
            vecdists.clear();
            for (int b = 0; b < cvec_in.size(); b++) {
                double vecdist = sqrt((avec_in[a] - cvec_in[b])*(avec_in[a] - cvec_in[b]) + (bvec_in[a] - dvec_in[b])*(bvec_in[a] - dvec_in[b]));
                vecdists.push_back(vecdist);
            }

            // Find minimum
            auto min = *min_element(vecdists.begin(), vecdists.end());
            vector<double>::iterator vind = find(vecdists.begin(), vecdists.end(), min);
            int vpos = distance(vecdists.begin(), vind);        

            // Push values to output vectors and delete old ones
            avec_out.push_back(cvec_in[vpos]);
            bvec_out.push_back(dvec_in[vpos]);

            cvec_in.erase(cvec_in.begin() + vpos);
            dvec_in.erase(dvec_in.begin() + vpos);
        }
    }
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

    // Remove the orange cone values from the vectors
    for (int ii = 0; ii < 13; ii++) {
        xg.pop_back();
        yg.pop_back();
    }

    // Split everything up into blue and yellow poses
    vector<double> xb, yb, xy, yy;
    if (xg.size() == yg.size()) {
        while ((!xg.empty()) && (!yg.empty())) {
            xb.push_back(xg[0]);
            xg.erase(xg.begin());
            yb.push_back(yg[0]);
            yg.erase(yg.begin());

            xy.push_back(xg[xg.size()/2-1]);
            xg.erase(xg.begin()+(xg.size()/2-1));
            yy.push_back(yg[xg.size()/2-1]);
            yg.erase(yg.begin()+(xg.size()/2-1));
        }
    }

    // Nearest neighbour ordering
    // Check: lists are the same size
    if((xb.size() != yb.size()) && (yb.size() != xy.size()) && (xy.size() != yy.size())) {
        ROS_ERROR("Coordinate lists are not the same size!!");
    }

    vector<double> xbl, ybl, xyl, yyl;
    xbl = vector_sort(xb);
    ybl = vector_sort(yb);
    xyl = vector_sort(xy);
    yyl = vector_sort(yy);

    // Sort the lists again
    vector<double> xyd, yyd;
    vector_dist_sort(xbl, ybl, xyl, yyl, xyd, yyd);

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
    // // Add the first pose
    // path_pose.pose.position.x = xp[0];
    // path_pose.pose.position.y = yp[0];
    // path_pose.pose.position.z = 0;
    // path_pose.pose.orientation.w = 0;
    // path_pose.pose.orientation.x = 0;
    // path_pose.pose.orientation.y = 0;
    // path_pose.pose.orientation.z = 0;        
    // path_msg.poses.push_back(path_pose);

    // // Remove it from the vector
    // xp.erase(xp.begin());
    // yp.erase(yp.begin());
    // vector<double>::iterator index;
    // int pos;
    
    // // Get distances from last path point to all other points, get minimum distance, add corresponding elements and remove
    // while((!xp.empty()) && (!yp.empty())) {
    //     // Euclidean distances
    //     vector<double> dists;
    //     for(int ii = 0; ii < xp.size(); ii++) {
    //         double dist = sqrt(((path_pose.pose.position.x - xp[ii])*(path_pose.pose.position.x - xp[ii])) + ((path_pose.pose.position.y - yp[ii])*(path_pose.pose.position.y - yp[ii])));
    //         dists.push_back(dist);
    //         ROS_INFO("Calculated distance for elements %d to be %0.4f", ii, dist);
    //     }

    //     // Find the minimum
    //     auto min = *min_element(dists.begin(), dists.end());
    //     index = find(dists.begin(), dists.end(), min);
    //     pos = distance(dists.begin(), index);
    //     ROS_INFO("Found distance at elements %d", pos);

    //     // Push these values into the path, and remove those values
    //     path_pose.pose.position.x = xp[pos];
    //     path_pose.pose.position.y = yp[pos];
    //     path_pose.pose.position.z = 0;
    //     path_pose.pose.orientation.w = 0;
    //     path_pose.pose.orientation.x = 0;
    //     path_pose.pose.orientation.y = 0;
    //     path_pose.pose.orientation.z = 0;        
    //     path_msg.poses.push_back(path_pose);

    //     xp.erase(xp.begin() + pos);
    //     yp.erase(yp.begin() + pos);        
    // }

    for (int ii = 0; ii < xp.size(); ii++) {
        // Pack the values to a pose, and add the pose to the path message
        path_pose.pose.position.x = xp[ii];
        path_pose.pose.position.y = yp[ii];
        path_pose.pose.position.z = 0;
        path_pose.pose.orientation.w = 0;
        path_pose.pose.orientation.x = 0;
        path_pose.pose.orientation.y = 0;
        path_pose.pose.orientation.z = 0;        
        path_msg.poses.push_back(path_pose);
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