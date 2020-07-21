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
#include <signal.h>

// Image pipeline node to handle all image processing tasks

using namespace std;
static const std::string OPENCV_WINDOW = "Image_Window";

class QEV_image {

    public:
    QEV_image();
    ~QEV_image();
    
    void image_left_callback(const sensor_msgs::Image::ConstPtr& img_left_msg);
    void image_right_callback(const sensor_msgs::Image::ConstPtr& img_right_msg);
    void colour_seg(void);

    private:
    ros::NodeHandle ni;
    image_transport::ImageTransport imi;
    image_transport::Subscriber im_left_sub;
    image_transport::Subscriber im_right_sub;
    ros::Publisher point_pub;
    ros::Publisher lap_pub; // Might be replaced with a param server function
    std_msgs::Int64 lap_msg;
    geometry_msgs::Point point_msg;


    // OpenCV variables 
    cv::Mat im_left, im_right, im_left_hsv, im_right_hsv, im_hsv_b;

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

void SigIntHandler(int sig) {
    // Close the cv window
    ROS_ERROR("Closing the display window...");
    cv::destroyWindow(OPENCV_WINDOW);
    ROS_INFO("Closing...");
    ros::shutdown();
}

// Constructor
QEV_image::QEV_image(): imi(ni) {

    // Publishers here
    point_pub = ni.advertise<geometry_msgs::Point>("qev2/move_point", 10);
    lap_pub = ni.advertise<std_msgs::Int64>("qev2/lap_count", 10);

    // Subscribers gere
    im_left_sub = imi.subscribe("/qev2/zed_camera/left/image_raw/compressed", 10, &QEV_image::image_left_callback, this);
    im_right_sub = imi.subscribe("/qev2/zed_camera/right/image_raw/compressed", 10, &QEV_image::image_right_callback, this);

    // Get a shutdown handler
    signal(SIGINT, SigIntHandler);

    // Create a window 
    cv::namedWindow(OPENCV_WINDOW);
}

void QEV_image::colour_seg(void) {
    // Segment the images to find yellow and blue 
    cv::cvtColor(im_left, im_left_hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(im_right, im_right_hsv, cv::COLOR_BGR2HSV);
}

void QEV_image::image_left_callback(const sensor_msgs::Image::ConstPtr& img_left_msg) {
    // Convert left image to an OpenCV friendly format
    cv_bridge::CvImagePtr cv_left_ptr;    

    // Try the conversion
    try {
        cv_left_ptr = cv_bridge::toCvCopy(img_left_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& exc) {
        ROS_ERROR("cv_bridge exception encountered: %s", exc.what());
        return;
    }

    // Save
    QEV_image::im_left = cv_left_ptr->image;
}

void QEV_image::image_right_callback(const sensor_msgs::Image::ConstPtr& img_right_msg) {
    // Convert right image to an OpenCV friendly format
    cv_bridge::CvImagePtr cv_right_ptr;

    // Try conversion
    try {
        cv_right_ptr = cv_bridge::toCvCopy(img_right_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& exc) {
        ROS_ERROR("cv_bridge exception encountered: %s", exc.what());
        return;
    }

    // Save
    QEV_image::im_right = cv_right_ptr->image;
}

int main(int argc, char **argv) {
    // Node init

    // Class object goes here


}