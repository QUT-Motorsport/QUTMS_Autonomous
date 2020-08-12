#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class Ls_to_PC2 {
     public:
        Ls_to_PC2();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

Ls_to_PC2::Ls_to_PC2(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/QEV2_lidar/lidar_scan", 100, &Ls_to_PC2::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/QEV2_lidar/pc2", 100, false);
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void Ls_to_PC2::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("QEV2_lidar/lidar_frame", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ls_to_pc2");

    Ls_to_PC2 ls_to_pc2;

    ros::spin();

    return 0;
}