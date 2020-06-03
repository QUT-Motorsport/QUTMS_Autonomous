#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>

// Parses sample GPS data and publishes at a certain rate to emulate a GPS sensor
using namespace std;

char dira[300];
char dirb[300];
char dirc[300];

int main(int argc, char **argv) {
    // Init
    ros::init(argc, argv, "KITTI_IMU_Pub");

    // Node handle
    ros::NodeHandle ni;

    // Publisher
    ros::Publisher qimu_pub = ni.advertise<sensor_msgs::Imu>("/KITTI_imu", 10);

    // Vector to hold data
    vector<double> imu_accel_data; 
    vector<double> imu_rate_data;
    vector<double> att_data;

    // Rate set
    ros::Rate rate(1); // Once per second

    // Get 2 directories to a text file
    sprintf(dira, "/home/nick-pc-ubuntu/Documents/Text Files/KITTI_accel_xyz.txt");
    sprintf(dirb, "/home/nick-pc-ubuntu/Documents/Text Files/KITTI_brate_xyz.txt");
    sprintf(dirc, "/home/nick-pc-ubuntu/Documents/Text Files/KITTI_att.txt");

    // Process the text file for acceleration data
    string aline;
    string::size_type asz;
    ifstream ainfile (dira);
    if(ainfile.is_open()) {
        while(getline(ainfile, aline)) {
            double num = stod(aline, &asz);
            imu_accel_data.push_back(num);
        }
    }
    ainfile.close();

    // Open up and process the next file
    string rline;
    string::size_type arz;
    ifstream rinfile (dirb);
    if(rinfile.is_open()) {
        while(getline(rinfile, rline)) {
            double num = stod(rline, &arz);
            imu_rate_data.push_back(num);
        }
    }
    rinfile.close();

    // Process attitude data
    string tline;
    string::size_type tsz;
    ifstream tinfile (dirc);
    if(tinfile.is_open()) {
        while(getline(tinfile, tline)) {
            double num = stod(tline, &tsz);
            att_data.push_back(num);
        }
    }
    tinfile.close();

    // Define a header
    std_msgs::Header imu_head;
    sensor_msgs::Imu imu_nav_msg;

    // Publish
    while(ros::ok()) {
        for (int ii = 0; ii < imu_accel_data.size()/3 - 2; ii++) {
            // Put in data to the message containers

            imu_head.frame_id = "IMU";
            imu_head.seq = ii;
            imu_head.stamp = ros::Time::now();

            imu_nav_msg.header = imu_head;

            // Start with attitude
            geometry_msgs::Quaternion attm;
            tf2::Quaternion quaternion;
            quaternion.setRPY(att_data[3*ii], att_data[3*ii+1], att_data[3*ii+2]);
            attm = tf2::toMsg(quaternion);
            imu_nav_msg.orientation = attm;

            // Acceleration data
            geometry_msgs::Vector3 accelvec;
            accelvec.x = imu_accel_data[3*ii];
            accelvec.y = imu_accel_data[3*ii+1];
            accelvec.z = imu_accel_data[3*ii+2];
            imu_nav_msg.linear_acceleration = accelvec;

            // Body rate data
            geometry_msgs::Vector3 bratevec;
            bratevec.x = imu_rate_data[3*ii];
            bratevec.y = imu_rate_data[3*ii+1];
            bratevec.z = imu_rate_data[3*ii+2];
            imu_nav_msg.angular_velocity = bratevec;

            // Publish
            qimu_pub.publish(imu_nav_msg);

            rate.sleep();

        }

    }
    

}