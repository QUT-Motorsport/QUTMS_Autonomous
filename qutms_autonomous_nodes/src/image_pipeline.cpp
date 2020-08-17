#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv_modules.hpp>
#include <signal.h>

// Image pipeline node to handle all image processing tasks
// IMPORTANT: use associated launch file for testing

using namespace std;
static const std::string OPENCV_WINDOW_LEFT = "Image_Window_Left";
static const std::string OPENCV_WINDOW_RIGHT = "Image_Window_Right";

class QEV_Image {

    public:
    QEV_Image();
    
    void image_left_callback(const sensor_msgs::Image::ConstPtr& img_left_msg);
    void image_right_callback(const sensor_msgs::Image::ConstPtr& img_right_msg);
    void colour_seg(void);
    void image_display(void);
    void im_left_save(cv::Mat im_left_in);
    void im_right_save(cv::Mat im_right_in);

    private:
    ros::NodeHandle ni;
    image_transport::ImageTransport imi;
    image_transport::Subscriber im_left_sub;
    image_transport::Subscriber im_right_sub;
    ros::Publisher point_pub;
    ros::Publisher lap_pub; // Might be replaced with a param server function
    std_msgs::Int64 lap_msg;
    geometry_msgs::Point point_msg;
    string image_transport_param;


    // OpenCV variables 
    cv::Mat im_left_r, im_right_r, im_left, im_right, im_left_hsv, im_right_hsv, im_hsv_b, im_thres_y_left, im_thres_y_right, im_thres_b_left, im_thres_b_right;

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
    ROS_ERROR("Closing the display windows...");
    cv::destroyWindow(OPENCV_WINDOW_LEFT);
    cv::destroyWindow(OPENCV_WINDOW_RIGHT);
    ROS_INFO("Closing...");
    ros::shutdown();
}

// Constructor
QEV_Image::QEV_Image(): imi(ni) {

    // Set image transport options to get compressed images
    // ni.setParam("/QEV_Image/image_transport", "compressed");
    // // DEBUG: Verify the parameter is appropriately set
    // ni.getParam("/QEV_Image/image_transport", image_transport_param);
    // cout << image_transport_param << endl;

    // Image transport hints
    // image_transport::TransportHints hints("compressed");

    // Publishers here
    point_pub = ni.advertise<geometry_msgs::Point>("/qev/move_point", 10);
    lap_pub = ni.advertise<std_msgs::Int64>("/qev/lap_count", 10);

    // Subscribers here
    im_left_sub = imi.subscribe("/qev/zed_camera/left/image_raw", 10, &QEV_Image::image_left_callback, this);
    im_right_sub = imi.subscribe("/qev/zed_camera/right/image_raw", 10, &QEV_Image::image_right_callback, this);

    // Get a shutdown handler
    signal(SIGINT, SigIntHandler);

    // Create a window 
    cv::namedWindow(OPENCV_WINDOW_LEFT);
    cv::namedWindow(OPENCV_WINDOW_RIGHT);

    // Startup the window thread
    cv::startWindowThread();
}

void QEV_Image::im_left_save(cv::Mat im_left_in) {
    // Save the left image
    im_left = im_left_in;
}

void QEV_Image::im_right_save(cv::Mat im_right_in) {
    // Save the right image
    im_right = im_right_in;
}

void QEV_Image::colour_seg(void) {
    // Segment the images to find yellow and blue 
    // Sanity check: we have an image
    if(im_left.empty() && im_right.empty()) {
        ROS_ERROR("No data in image files");
    }

    cv::cvtColor(im_left, im_left_hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(im_right, im_right_hsv, cv::COLOR_BGR2HSV);

    // Threshold yellow in each image
    cv::inRange(im_left_hsv, cv::Scalar(y_low_h,y_low_s,y_low_v), cv::Scalar(y_high_h,y_high_s,y_high_v), im_thres_y_left);
    cv::inRange(im_right_hsv, cv::Scalar(y_low_h,y_low_s,y_low_v), cv::Scalar(y_high_h,y_high_s,y_high_v), im_thres_y_right);

    // Threshold blue
    cv::inRange(im_left_hsv, cv::Scalar(b_low_h,b_low_s,b_low_v), cv::Scalar(b_high_h, b_high_s, b_high_v),im_thres_b_left);
    cv::inRange(im_right_hsv, cv::Scalar(b_low_h,b_low_s,b_low_v), cv::Scalar(b_high_h, b_high_s, b_high_v),im_thres_b_left);

    // Moments
    cv::Moments im_left_ymoments = cv::moments(im_thres_y_left);
    cv::Moments im_right_ymoments = cv::moments(im_thres_y_right);
    cv::Moments im_left_bmoments = cv::moments(im_thres_b_left);
    cv::Moments im_right_bmoments = cv::moments(im_thres_b_right);

}

void QEV_Image::image_display(void) {
    // For debug purposes: displays images for checking
    // Resize images to fit screens
    cv::resize(im_thres_y_left, im_thres_y_left, cv::Size(900, 600));
    cv::resize(im_thres_y_right, im_thres_y_right, cv::Size(900, 600));
    cv::resize(im_thres_b_left, im_thres_b_left, cv::Size(900, 600));
    cv::resize(im_thres_b_right, im_thres_b_right, cv::Size(900, 600));

    // // Display the images
    cv::Mat im_thres_left = im_thres_b_left + im_thres_y_left;
    cv::Mat im_thres_right = im_thres_b_right + im_thres_y_right;

    cv::imshow(OPENCV_WINDOW_LEFT, im_thres_left);
    cv::imshow(OPENCV_WINDOW_RIGHT, im_thres_right); 
}

void QEV_Image::image_left_callback(const sensor_msgs::Image::ConstPtr& img_left_msg) {
    // Convert left image to an OpenCV friendly format
    cv_bridge::CvImagePtr cv_left_ptr;    

    // Try the conversion
    try {
        cv_left_ptr = cv_bridge::toCvCopy(img_left_msg, sensor_msgs::image_encodings::BGR16);
    } catch (cv_bridge::Exception& exc) {
        ROS_ERROR("cv_bridge exception encountered: %s", exc.what());
        return;
    }

    // Save
    QEV_Image::im_left_r = cv_left_ptr->image;
    im_left_save(QEV_Image::im_left_r);
    // cv::imshow(OPENCV_WINDOW_LEFT, im_left);
}

void QEV_Image::image_right_callback(const sensor_msgs::Image::ConstPtr& img_right_msg) {
    // Convert right image to an OpenCV friendly format
    cv_bridge::CvImagePtr cv_right_ptr;

    // Try conversion
    try {
        cv_right_ptr = cv_bridge::toCvCopy(img_right_msg, sensor_msgs::image_encodings::BGR16);
    } catch (cv_bridge::Exception& exc) {
        ROS_ERROR("cv_bridge exception encountered: %s", exc.what());
        return;
    }

    // Save
    QEV_Image::im_right_r = cv_right_ptr->image;
    im_right_save(QEV_Image::im_right_r);
    // cv::imshow(OPENCV_WINDOW_RIGHT, im_right);

}

int main(int argc, char **argv) {
    // Node init
    ros::init(argc, argv, "QEV_Image", ros::init_options::NoSigintHandler);

    // Class object goes here
    QEV_Image qev_image;

    // Rate set
    ros::Rate rate(1);

    // Enter while loop here
    while(ros::ok()) {
        // Spin to get some images
        ros::spinOnce();

        // Process the images
        qev_image.colour_seg();

        // // Display
        // qev_image.image_display();

        rate.sleep();
    }

}