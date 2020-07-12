#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

// Image pipeline node to handle all image processing tasks

using namespace std;
using namespace cv;

class QEV_image {

    public:
    QEV_image();

    private:
    ros::NodeHandle ni;
    ros::Subscriber im_sub;
    ros::Publisher point_pub;
    ros::Publisher lap_pub; // Might be replaced with a param server function
    std_msgs::Int64 lap_msg;
    geometry_msgs::Point point_msg;

    // OpenCV variables 
    Mat im_org, im_hsv, im_hsv_b;

    // Image segmentation values
    // Yellow values
    int y_low_h=8;
    int y_high_h=38;
    int y_low_s=176;
    int y_high_s=255;
    int y_low_v=108;
    int y_high_v=155;

    int y_iLastX = -1; 
    int y_iLastY = -1;

    // Blue values

    int b_low_h=114;
    int b_high_h=125;
    int b_low_s=180;
    int b_high_s=255;
    int b_low_v=17;
    int b_high_v=117;

    int b_iLastX = -1; 
    int b_iLastY = -1;


};

int main(int argc, char **argv) {
    // Node init

    // Class object goes here


}