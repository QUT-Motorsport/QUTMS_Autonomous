#include <algorithm>
#include <assert.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

// Includes for custom messages
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

// Includes for pointcloud processing
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <boost/bind.hpp>

#include "tf/transform_datatypes.h"

typedef pcl::PointXYZ PointType;

ros::Publisher rosPubFilter;
ros::Publisher rosPubMarkers;
visualization_msgs::MarkerArray markersMsg;

std::string inputTopic;
std::string outputTopic;

boost::mutex lock;

// Variables for pass through filter
double filterZMin;              // Minimal range (m) in z-axis to filter the ground
double filterZMax;              // Maximal range (m) in z-axis

// Variables for voxel grid filter
double leafSize;

// Variables for clustering
double rangeMax;            // Maximal scanning range in x-y plane
double openingDegreeMax;    // In rad
double openingDegreeMin;    // In rad
double thresholdRadius;     // Clustering treshold radius
int noiseThreshold;

// Variables for LShapeFitting
bool lShapeFittingEnabled = true;
int amountOfSteps;                      // Check every angle position (0°, 1°, 2°,  ...)
double resolution;                      //Angelresolution in deg
double degreeThreshold;

bool debugMode;
double updateRate;

bool GetParameters(ros::NodeHandle &nh)
{
    // Check for input topic to receive pointcloud of LIDAR
    if (nh.hasParam("inputTopic"))
    {
        nh.getParam("inputTopic", inputTopic);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <inputTopic> was not set, defaulted to /qev2/horizon_lidar/pointcloud");
        inputTopic = "/qev2/horizon_lidar/pointcloud";
    }

    // Check for output topic to publish filtered pointcloud
    if (nh.hasParam("outputTopic"))
    {
        nh.getParam("outputTopic", outputTopic);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <outputTopic> was not set, defaulted to /lidar_clustering_node/pointcloud_filtered");
        outputTopic = "/lidar_clustering_node/pointcloud_filtered";
    }

    // Check for custom filterZMax
    if (nh.hasParam("filterZMax"))
    {
        nh.getParam("filterZMax", filterZMax);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <filterZMax> of: " << filterZMax);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <filterZMax> was not set, defaulted to 5.0.");
        filterZMax = 5.0;
    }

    // Check for custom filterZMin
    if (nh.hasParam("filterZMin"))
    {
        nh.getParam("filterZMin", filterZMin);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <filterZMin> of: " << filterZMin);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <filterZMin> was not set, defaulted to -0.075.");
        filterZMin = -0.075;
    }

    // Check for custom leafSize
    if (nh.hasParam("leafSize"))
    {
        nh.getParam("leafSize", leafSize);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <leafSize> of: " << leafSize);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <leafSize> was not set, defaulted to 0.01.");
        leafSize = 0.01;
    }

    // Check for custom rangeMax
    if (nh.hasParam("rangeMax"))
    {
        nh.getParam("rangeMax", rangeMax);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <rangeMax> of: " << rangeMax);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <leafSize> was not set, defaulted to 25.");
        rangeMax = 25;
    }

    // Check for custom openingDegreeMax
    if (nh.hasParam("openingDegreeMax"))
    {
        nh.getParam("openingDegreeMax", openingDegreeMax);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <openingDegreeMax> of: " << openingDegreeMax);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <openingDegreeMax> was not set, defaulted to M_PI.");
        openingDegreeMax = M_PI;
    }

    // Check for custom openingDegreeMin
    if (nh.hasParam("openingDegreeMin"))
    {
        nh.getParam("openingDegreeMin", openingDegreeMin);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <openingDegreeMin> of: " << openingDegreeMin);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <openingDegreeMin> was not set, defaulted to 0.0.");
        openingDegreeMin = 0.0;
    }

    // Check for custom thresholdRadius
    if (nh.hasParam("thresholdRadius"))
    {
        nh.getParam("thresholdRadius", thresholdRadius);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <thresholdRadius> of: " << thresholdRadius);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <thresholdRadius> was not set, defaulted to 0.1.");
        thresholdRadius = 0.1;
    }

    // Check for custom thresholdRadius
    if (nh.hasParam("noiseThreshold"))
    {
        nh.getParam("noiseThreshold", noiseThreshold);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <noiseThreshold> of: " << noiseThreshold);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <noiseThreshold> was not set, defaulted to 10.");
        noiseThreshold = 10;
    }

    // Check for custom lShapeFittingEnabled
    if (nh.hasParam("lShapeFittingEnabled"))
    {
        nh.getParam("lShapeFittingEnabled", lShapeFittingEnabled);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <lShapeFittingEnabled> of: " << lShapeFittingEnabled);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <lShapeFittingEnabled> was not set, defaulted to false.");
        lShapeFittingEnabled = false;
    }

    // Check for custom amountOfSteps
    if (nh.hasParam("amountOfSteps"))
    {
        nh.getParam("amountOfSteps", amountOfSteps);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <amountOfSteps> of: " << amountOfSteps);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <amountOfSteps> was not set, defaulted to 90.");
        amountOfSteps = 90;
    }

    // Initialize resolution according to amountOfSteps
    resolution = M_PI / 2. / double(amountOfSteps); //Angelresolution in deg

    // Check for custom degreeThreshold
    if (nh.hasParam("degreeThreshold"))
    {
        nh.getParam("degreeThreshold", degreeThreshold);
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node started with <degreeThreshold> of: " << degreeThreshold);
    }
    else
    {
        ROS_INFO_STREAM_NAMED("LidarClustering", "LidarClustering node <degreeThreshold> was not set, defaulted to 90.");
        degreeThreshold = 0.001;
    }

    // Check if debugMode is selected or not
    if (nh.hasParam("debugMode"))
    {
        nh.getParam("debugMode", debugMode);
        ROS_INFO_STREAM_NAMED("DriveToGoal", "DriveToGoal node started with <debugMode> set to: " << debugMode);
        if (debugMode == true)
        {
            ROS_INFO_NAMED("DriveToGoal", "DriveToGoal node Publishing debugging markers to: /drive_to_goal_node/debug_markers");
        }
    }
    else
    {
        debugMode = false;
    }

    return true;
}

visualization_msgs::Marker BoundBoxFitting(pcl::PointCloud<PointType>::Ptr &cluster)
{
    int amountPoints = int(cluster->size());

    double maxDistance = 0;
    double currentDistance = 0;
    double centerPoint[2];
    double distanceX;
    double distanceY;

    // Iterate over points of cluster and calculate the biggest distance of two points in x-y plane and their center
    for (int i = 0; i < amountPoints - 1; i++)
    {
        for (int j = i + 1; j < amountPoints; j++)
        {
            distanceX = cluster->points[i].x - cluster->points[j].x;
            distanceY = cluster->points[i].y - cluster->points[j].y;
            currentDistance = std::sqrt(std::pow(distanceX ,2) + std::pow(distanceY ,2));
            if (currentDistance > maxDistance)
            {
                maxDistance = currentDistance;
                centerPoint[0] = cluster->points[i].x + (distanceX / 2);
                centerPoint[1] = cluster->points[i].y - (distanceY / 2);
            }
        }
    }

    // Create a marker for visualization purpose
    visualization_msgs::Marker marker;
    
    marker.scale.x = maxDistance;
    marker.scale.y = maxDistance;
    marker.scale.z = 1.0;
    marker.pose.position.x = centerPoint[0];
    marker.pose.position.y = centerPoint[1];
    marker.pose.position.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}

void OnMsgCallback(ros::NodeHandle &nh, const sensor_msgs::PointCloud2ConstPtr &_msg)
{
    // Lock mutex (locks access to fields used in message callbacks)
    lock.lock();

    // Convert ROS msg PointCloud2 type to PCL PointCloud2 type
    pcl::PCLPointCloud2 pointCloud2;
    pcl_conversions::toPCL(*_msg, pointCloud2);

    // Convert PCL PointCloud2 type to PCL PointCloudPointXYZ type
    pcl::PointCloud<PointType>::Ptr unfilteredPointCloud(new pcl::PointCloud<PointType>);
    pcl::fromPCLPointCloud2(pointCloud2, *unfilteredPointCloud);

#pragma region Pass Through Filter

    // Pass Through Filter
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(unfilteredPointCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(filterZMin, filterZMax);

    double z = 0;

    for (int i = 0; i < unfilteredPointCloud->points.size(); i++)
    {
        if (unfilteredPointCloud->points[i].z < z)
        {
            z = unfilteredPointCloud->points[i].z;
        }
    }

    pass.filter(*unfilteredPointCloud);

#pragma endregion Pass Through Filter

#pragma region Voxel Grid Filter

    // Only downsample and cluster if amount of points in pointcloud is larger than noiseThreshold
    if (unfilteredPointCloud->points.size() > noiseThreshold)
    {
        // Voxel grid downsampling
        if (leafSize > 0)
        {
            pcl::VoxelGrid<PointType> sor;
            sor.setInputCloud(unfilteredPointCloud);
            sor.setLeafSize(leafSize, leafSize, leafSize);
            sor.filter(*unfilteredPointCloud);
        }

#pragma endregion Voxel Grid Filter

#pragma region Clustering    

        // Clustering

        // Initialize vector holding all points of filtered unclustered points
        std::vector<PointType, Eigen::aligned_allocator<PointType>> pointvect;
        pointvect = unfilteredPointCloud->points;

        // Cluster algorithm variables
        pcl::KdTreeFLANN<PointType> kdtree;
        std::vector<int> inRangePointIds;
        std::vector<float> pointRadiusSquaredDistance;
        std::vector<PointType, Eigen::aligned_allocator<PointType>> clusterQueue;
        std::vector<PointType, Eigen::aligned_allocator<PointType>> clusterList;
        std::vector<std::vector<PointType, Eigen::aligned_allocator<PointType>>> objectList;
        std::vector<int> processedPointsMarker(pointvect.size());

        // Sort Point Cloud Data into k-d-tree
        kdtree.setInputCloud(unfilteredPointCloud);

        // Clustering Algorithm:
        for (int i = 0; i < pointvect.size(); i++)
        {
            if (processedPointsMarker[i] != 1)
            {
                processedPointsMarker[i] = 1;
                clusterQueue.push_back(pointvect[i]);
                while (clusterQueue.size() > 0)
                {
                    float radius = sqrt(pow((clusterQueue[0]).x, 2) + pow((clusterQueue[0]).y, 2) + pow((clusterQueue[0]).z, 2));

                    // Search k-d-tree for neighbours in radius
                    if (kdtree.radiusSearch(clusterQueue[0], radius * thresholdRadius, inRangePointIds, pointRadiusSquaredDistance) > 0)
                    {
                        for (int k = 0; k < inRangePointIds.size(); ++k)
                        {
                            if (processedPointsMarker[inRangePointIds[k]] != 1)
                            {
                                clusterQueue.push_back(pointvect[inRangePointIds[k]]);
                                processedPointsMarker[inRangePointIds[k]] = 1;
                            }
                        }
                    }
                    clusterList.push_back(clusterQueue[0]);
                    clusterQueue.erase(clusterQueue.begin());
                }
                if (clusterList.size() > noiseThreshold)
                {
                    objectList.push_back(clusterList);
                }
                clusterList.clear();
            }
        }

#pragma endregion Clustering

#pragma region BoundBoxFitting

        // Read out current cluster of objectList
        pcl::PointCloud<PointType>::Ptr currentCluster(new pcl::PointCloud<PointType>);

        // Create msg of type MarkerArray for visualization of boundingBox in rviz
        visualization_msgs::MarkerArray markersMsg;

        // Loop through all clusters
        for (int i = 0; i < objectList.size(); i++)
        {
            // Read out current cluster of objectList
            currentCluster->points = objectList[i];

            // Get Cylinder as bounding box
            visualization_msgs::Marker boundingCylinder = BoundBoxFitting(currentCluster);

            // Adjust parameters
            boundingCylinder.type = visualization_msgs::Marker::CYLINDER;
            boundingCylinder.id = i;

            // Get details of header from origingal message
            boundingCylinder.header = _msg->header;
            markersMsg.markers.push_back(boundingCylinder);
        }

        // Publish markers
        rosPubMarkers.publish(markersMsg);

        

#pragma endregion BoundBoxFitting

    }

    // ---- Publish filtered and downsampled pointcloud ----

    // Convert PCL PointCloudPointXYZ to PCL PointCloud2
    pcl::PCLPointCloud2 filteredPC2;
    pcl::toPCLPointCloud2(*unfilteredPointCloud, filteredPC2);

    // Convert PCL PointCloud2 to ROS msg PointCloud2 type
    sensor_msgs::PointCloud2 filteredPC2Msg;
    pcl_conversions::fromPCL(filteredPC2, filteredPC2Msg);

    rosPubFilter.publish(filteredPC2Msg);

    // Unlock mutex
    lock.unlock();
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "debug_clustering");

    ros::NodeHandle nh("~");

    if (GetParameters(nh) != true)
    {
        return 0;
    }

    // Initialize subscriber and publisher with input/output topics
    ros::Subscriber rosSubPC = nh.subscribe<sensor_msgs::PointCloud2>(inputTopic, 1, boost::bind(&OnMsgCallback, boost::ref(nh), _1));
    rosPubFilter = nh.advertise<sensor_msgs::PointCloud2>(outputTopic, 1);


    rosPubMarkers = nh.advertise<visualization_msgs::MarkerArray>("/lidar_clustering_node/bounding_boxes", 1);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
