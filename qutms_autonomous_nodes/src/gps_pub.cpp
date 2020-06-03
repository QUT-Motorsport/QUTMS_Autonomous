#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>

// Parses sample GPS data and publishes at a certain rate to emulate a GPS sensor
using namespace std;

char dir[300];

int main(int argc, char **argv) {
    // Init
    ros::init(argc, argv, "KITTI_GPS_Pub");

    // Node handle
    ros::NodeHandle ng;

    // Publisher
    ros::Publisher qgps_pub = ng.advertise<sensor_msgs::NavSatFix>("/KITTI_gps", 10);

    // Vector to hold data
    vector<double> gps_data; 

    // Rate set
    ros::Rate rate(1); // Once per second

    // Get a directory to a text file
    sprintf(dir, "/home/nick-pc-ubuntu/Documents/Text Files/KITTI_gps.txt");

    // Process the text file
    string line;
    string::size_type sz;
    ifstream infile (dir);
    if(infile.is_open()) {
        while(getline(infile, line)) {
            double num = stod(line, &sz);
            gps_data.push_back(num);
        }
    }
    infile.close();

    // Get the vector size
    int vsz = gps_data.size();

    // Define a header
    std_msgs::Header gps_head;
    sensor_msgs::NavSatStatus gps_stat;
    sensor_msgs::NavSatFix gps_nav_msg;

    // Publish
    while(ros::ok()) {
        for(int ii = 0; ii < (vsz/3)-2; ii++) {
        // Header
        gps_head.frame_id = "GPS";
        gps_head.seq = ii;
        gps_head.stamp = ros::Time::now();

        gps_nav_msg.header = gps_head;

        // Fill the data message
        gps_nav_msg.latitude = gps_data[3*ii];
        gps_nav_msg.longitude = gps_data[3*ii+1];
        gps_nav_msg.altitude = gps_data[3*ii+2];

        // Publish
        qgps_pub.publish(gps_nav_msg);

        rate.sleep();

        }
    }
    

}