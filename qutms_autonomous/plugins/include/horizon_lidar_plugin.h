#ifndef GAZEBO_ROS_HORIZON_LASER_HH
#define GAZEBO_ROS_HORIZON_LASER_HH

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <tf/tf.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>

#include <sensor_msgs/PointCloud2.h>

namespace gazebo
{
  class GazeboRosHorizonLaser : public RayPlugin
  {
    /// \brief Constructor
    
    public: GazeboRosHorizonLaser();

    public: ~GazeboRosHorizonLaser();

    /// \brief Load the plugin
    /// \param take in SDF root element
    /// \param parent The parent entity, must be a Model or a Sensor
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Subscribe on-demand
    private: void ConnectCb();

    /// \brief The parent ray sensor
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief Pointer to ROS node
    private: ros::NodeHandle* nh_;

    /// \brief ROS publisher
    private: ros::Publisher pub_;

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief the intensity beneath which points will be filtered
    private: double min_intensity_;

    /// \brief A mutex to lock access
    private: boost::mutex lock_;

    /// \brief For setting ROS name space
    private: std::string robot_namespace_;

    // Custom Callback Queue
    private: ros::CallbackQueue laser_queue_;
    private: void laserQueueThread();
    private: boost::thread callback_laser_queue_thread_;

    // Subscribe to gazebo laserscan
    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr sub_;
    private: void OnScan(const ConstLaserScanStampedPtr &_msg);
  };
}

#endif