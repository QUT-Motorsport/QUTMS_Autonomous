#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/opencv.hpp>
#include <math.h>

class LiDAR_Pipeline {
    public:
    LiDAR_Pipeline();

    private:
    void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);

    // ROS variables and other bits
    ros::Publisher cl_pub;
    ros::Subscriber pcl_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    ros::NodeHandle nlp;
    geometry_msgs::Point cone_point_msg;

    // Other things here
    std::string as_state;
    double xp;
    double yp;
    double zp;
    double xv;
    double yv;
    double time_then;
    double time_now;

};

LiDAR_Pipeline::LiDAR_Pipeline() {
    // Get the system state
    if(nlp.getParam("AS_State", as_state)) {
        //nlp.getParam("AS_State", as_state);
        ROS_INFO("Activating LiDAR pipeline...");
    }

    // Initialise start up time
    time_now = ros::Time::now().toSec();

    // Publisher
    cl_pub = nlp.advertise<geometry_msgs::Point>("/cone_position", 10);

    // Subscribers
    pcl_sub = nlp.subscribe("/qev2/horizon_lidar/points", 5, &LiDAR_Pipeline::pcl_callback, this);
    gps_sub = nlp.subscribe("/qev2/gps/data", 5, LiDAR_Pipeline::gps_callback, this);
    imu_sub = nlp.subscribe("/qev2/imu/data", 5, LiDAR_Pipeline::imu_callback, this);
}

void LiDAR_Pipeline::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
    // Get values
    double lat = gps_msg->latitude;
    double lon = gps_msg->longitude;
    double alt = gps_msg->altitude;

    // Go to radians
    lat = (M_PI/180)*lat;
    lon = (M_PI/180)*lon;

    // Get position as Euclidean
    xp = alt*sin(lat)*cos(lon);
    yp = alt*sin(lat)*sin(lon);
    zp = alt*cos(lat);
}

void LiDAR_Pipeline::pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg) {

}

void LiDAR_Pipeline::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // Get acceleration values
    geometry_msgs::Vector3 accel = imu_msg->linear_acceleration;

    // Get time after accessing measurement 
    time_then = ros::Time::now().toSec();
    
    // Get velocity
    xv = accel.x*(time_then - time_now); // m/s/s * => m/s
    yv = accel.y*(time_then - time_now);

    // Get time at close of calculation
    time_now = ros::Time::now().toSec();

}

int main(int argc, char **argv) {
    // Init
    ros::init(argc, argv, "LiDAR_Processor");

    // Run and spin
    LiDAR_Pipeline lidar_pipeline;

    ros::spin();

    return 0;
}