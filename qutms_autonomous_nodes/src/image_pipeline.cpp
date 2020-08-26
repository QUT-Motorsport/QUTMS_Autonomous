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
// #include "opencv2/imgproc.hpp"
// #include "opencv2/highgui.hpp"
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

    // Cluster point values
    int b_left_x, b_left_y, y_left_x, y_left_y, b_right_x, b_right_y, y_right_x, y_right_y;

    // Image segmentation values
    // Yellow values
    int y_low_h=8;
    int y_high_h=38;
    int y_low_s=206;
    int y_high_s=255;
    int y_low_v=158;
    int y_high_v=185;

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

void QEV_Image::image_left_callback(const sensor_msgs::Image::ConstPtr& img_left_msg) {
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
    cv::Mat im_left = cv_left_ptr->image;
    
    // Go to HSV
    cv::Mat im_left_hsv;
    cv::cvtColor(im_left, im_left_hsv, cv::COLOR_BGR2HSV);

    // Threshold yellow
    cv::Mat im_thres_y_left, im_thres_b_left;
    cv::inRange(im_left_hsv, cv::Scalar(y_low_h,y_low_s,y_low_v), cv::Scalar(y_high_h,y_high_s,y_high_v), im_thres_y_left);

    // Threshold blue
    cv::inRange(im_left_hsv, cv::Scalar(b_low_h,b_low_s,b_low_v), cv::Scalar(b_high_h, b_high_s, b_high_v), im_thres_b_left);

    // Moments
    cv::Moments im_left_ymoments = cv::moments(im_thres_y_left);
    cv::Moments im_left_bmoments = cv::moments(im_thres_b_left);

    // Recreate an image
    cv::Mat im_thres_left = im_thres_b_left + im_thres_y_left;
    
    // Reindex 
    im_thres_left = im_thres_left(cv::Range(600, 1080), cv::Range(1020, 2580));

    // Remove small objects
    cv::erode(im_thres_left, im_thres_left, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(im_thres_left, im_thres_left, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));     

    // Check for blue and yellow detections
    int b_left_cnt = 0; 
    int y_left_cnt = 0;
    for(int ii = 0; ii < im_thres_left.rows; ii++) {
        for(int jj = 0; jj < im_thres_left.cols/2; jj++) {
            // Check half the image for blue values
            if(im_thres_left.at<uchar>(ii,jj) > 0) {
                b_left_cnt++;
            }
        }

        for(int n = (im_thres_left.cols/2)+1; n < im_thres_left.cols; n++) {
            // Check the other half for yellow values
            if(im_thres_left.at<uchar>(ii,n) > 0) {
                y_left_cnt++;
            }
        }
    }

    // Notify if no detections are present
    if(b_left_cnt == 0) {
        ROS_ERROR("No blue cones detected!!!");
    }
    
    if(y_left_cnt == 0) {
        ROS_ERROR("No yellow cones detected!!!");
    }

    // Grab the image moment values
    double im_b_left_m01 = im_left_bmoments.m01;
    double im_b_left_m10 = im_left_bmoments.m10;
    double im_b_left_area = im_left_bmoments.m00;
    double im_y_left_m01 = im_left_ymoments.m01;
    double im_y_left_m10 = im_left_ymoments.m10;
    double im_y_left_area = im_left_ymoments.m00;

    // Get only the significant moments and calculate cluster positions
    // DEBUG
    // cout << "m00 for blue and yellow are: " << im_b_left_area << " " << im_y_left_area << endl;
    if(im_b_left_area > 100000 && im_y_left_area > 100000) {
        // Calculate cluster points
        b_left_x = im_b_left_m10/im_b_left_area;
        b_left_y = im_b_left_m01/im_b_left_area;
        y_left_x = im_y_left_m10/im_y_left_area;
        y_left_y = im_y_left_m01/im_y_left_area;
    }
    // ROS_INFO("Calculated left point pairs are: (%d, %d), (%d, %d)", b_left_x, b_left_y, y_left_x, y_left_y);

    // Display
    cv::imshow(OPENCV_WINDOW_LEFT, im_thres_left);

}

void QEV_Image::image_right_callback(const sensor_msgs::Image::ConstPtr& img_right_msg) {
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
    cv::Mat im_right = cv_right_ptr->image;

    // Go to HSV
    cv::Mat im_right_hsv;
    cv::cvtColor(im_right, im_right_hsv, cv::COLOR_BGR2HSV);

    // Threshold yellow
    cv::Mat im_thres_y_right, im_thres_b_right;
    cv::inRange(im_right_hsv, cv::Scalar(y_low_h,y_low_s,y_low_v), cv::Scalar(y_high_h,y_high_s,y_high_v), im_thres_y_right);

    // Threshold blue
    cv::inRange(im_right_hsv, cv::Scalar(b_low_h,b_low_s,b_low_v), cv::Scalar(b_high_h, b_high_s, b_high_v),im_thres_b_right);
    
    // Moments
    cv::Moments im_right_ymoments = cv::moments(im_thres_y_right);
    cv::Moments im_right_bmoments = cv::moments(im_thres_b_right);

    // Rejoin the images
    cv::Mat im_thres_right = im_thres_b_right + im_thres_y_right;

    // Reindex 
    im_thres_right = im_thres_right(cv::Range(600, 1080), cv::Range(1020, 2580));

    // Remove small objects
    cv::erode(im_thres_right, im_thres_right, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(im_thres_right, im_thres_right, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))); 

    // Check for blue and yellow detections
    int b_right_cnt = 0; 
    int y_right_cnt = 0;
    for(int ii = 0; ii < im_thres_right.rows; ii++) {
        for(int jj = 0; jj < im_thres_right.cols/2; jj++) {
            // Check half the image for blue values
            if(im_thres_right.at<uchar>(ii,jj) > 0) {
                b_right_cnt++;
            }
        }
        
        for(int n = (im_thres_right.cols/2)+1; n < im_thres_right.cols; n++) {
            // Check the other half for yellow values
            if(im_thres_right.at<uchar>(ii,n) > 0) {
                y_right_cnt++;
            }
        }
    }

    // Notify if no detections are present
    if(b_right_cnt == 0) {
        ROS_ERROR("No blue cones detected!!!");
    }
    
    if(y_right_cnt == 0) {
        ROS_ERROR("No yellow cones detected!!!");
    }
    
    // Grab the image moment values
    double im_b_right_m01 = im_right_bmoments.m01;
    double im_b_right_m10 = im_right_bmoments.m10;
    double im_b_right_area = im_right_bmoments.m00;
    double im_y_right_m01 = im_right_ymoments.m01;
    double im_y_right_m10 = im_right_ymoments.m10;
    double im_y_right_area = im_right_ymoments.m00;

    // Get only the significant moments and calculate cluster positions
    // DEBUG
    // cout << "m00 for blue and yellow are: " << im_b_right_area << " " << im_y_right_area << endl;
    if(im_b_right_area > 100000 && im_y_right_area > 100000) {
        // Calculate cluster points
        b_right_x = im_b_right_m10/im_b_right_area;
        b_right_y = im_b_right_m01/im_b_right_area;
        y_right_x = im_y_right_m10/im_y_right_area;
        y_right_y = im_y_right_m01/im_y_right_area;
    }
    // ROS_INFO("Calculated right point pairs are: (%d, %d), (%d, %d)", b_right_x, b_right_y, y_right_x, y_right_y);

    // Display
    cv::imshow(OPENCV_WINDOW_RIGHT, im_thres_right); 

}

int main(int argc, char **argv) {
    // Node init
    ros::init(argc, argv, "QEV_Image", ros::init_options::NoSigintHandler);

    // Class object goes here
    QEV_Image qev_image;

    // Rate set
    // ros::Rate rate(1);

    // // Enter while loop hereim_left_moments
    // while(ros::ok()) {
    //     // Spin to get some images
    //     ros::spinOnce();

    //     rate.sleep();
    // }

    ros::spin();

}