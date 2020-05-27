#include <horizon_lidar_plugin.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosHorizonLaser)

// Constructor
GazeboRosHorizonLaser::GazeboRosHorizonLaser() : nh_(NULL)
{
}

// Destructor
GazeboRosHorizonLaser::~GazeboRosHorizonLaser()
{
  // Finalize the controller / Custom Callback Queue
  this->laser_queue_.clear();
  this->laser_queue_.disable();
  if (this->nh_) {
    this->nh_->shutdown();
    delete this->nh_;
    this->nh_ = NULL;
  }
  this->callback_laser_queue_thread_.join();
}

// Load the controller
void GazeboRosHorizonLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, _sdf);

  // Initialize Gazebo node
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init();

  this->parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
  {
    gzthrow("Horizon laser plugin controller requires a Ray Sensor as its parent");
  }

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!_sdf->HasElement("frameName")) 
  {
    ROS_INFO("Horizon laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  } 
  else 
  {
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  this->min_intensity_ = std::numeric_limits<double>::lowest();
  if (!_sdf->HasElement("min_intensity")) 
  {
    ROS_INFO("Horizon laser plugin missing <min_intensity>, defaults to no clipping");
  } 
  else 
  {
    this->min_intensity_ = _sdf->GetElement("min_intensity")->Get<double>();
  }

  if (!_sdf->HasElement("topicName")) 
  {
    ROS_INFO("Horizon laser plugin missing <topicName>, defaults to /points");
    this->topic_name_ = "/points";
  } 
  else 
  {
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->nh_ = new ros::NodeHandle(this->robot_namespace_);

  // Resolve tf prefix
  std::string prefix;
  this->nh_->getParam(std::string("tf_prefix"), prefix);
  boost::trim_right_if(prefix, boost::is_any_of("/"));
  this->frame_name_ = tf::resolve(prefix, this->frame_name_);

  // Advertise publisher with a custom callback queue
  if (this->topic_name_ != "") {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
        this->topic_name_, 1,
        boost::bind(&GazeboRosHorizonLaser::ConnectCb, this),
        boost::bind(&GazeboRosHorizonLaser::ConnectCb, this),
        ros::VoidPtr(), &this->laser_queue_);
    this->pub_ = nh_->advertise(ao);
  }
  
  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);

  // Start custom queue for laser
  this->callback_laser_queue_thread_ = boost::thread( boost::bind( &GazeboRosHorizonLaser::laserQueueThread, this ) );

  ROS_INFO("Horizon laser plugin ready");
  
}

// Subscribe on-demand
void GazeboRosHorizonLaser::ConnectCb()
{
  boost::lock_guard<boost::mutex> lock(this->lock_);
  if (this->pub_.getNumSubscribers()) 
  {
    if (!this->sub_) 
    {
      this->sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &GazeboRosHorizonLaser::OnScan, this);
    }
    this->parent_ray_sensor_->SetActive(true);
  } 
  else 
  {
    if (this->sub_) 
    {
      this->sub_->Unsubscribe();
      this->sub_.reset();
    }
    this->parent_ray_sensor_->SetActive(false);
  }
}

void GazeboRosHorizonLaser::OnScan(ConstLaserScanStampedPtr& _msg)
{
  const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
  const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();

  const double maxRange = parent_ray_sensor_->RangeMax();
  const double minRange = parent_ray_sensor_->RangeMin();

  const int rayCount = parent_ray_sensor_->RayCount();
  const int rangeCount = parent_ray_sensor_->RangeCount();

  const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();

  const ignition::math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
  const ignition::math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();

  const double yDiff = maxAngle.Radian() - minAngle.Radian();
  const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  // Populate message fields
  const uint32_t POINT_STEP = 32;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

  int i, j;
  uint8_t *ptr = msg.data.data();
  for (i = 0; i < rangeCount; i++) 
  {
    for (j = 0; j < verticalRangeCount; j++) 
    {
      // Range
      double r = _msg->scan().ranges(i + j * rangeCount);
      // Intensity
      double intensity = _msg->scan().intensities(i + j * rangeCount);

      // Ignore points that are beneath a minimum intensity level
      if ( intensity < this->min_intensity_) {
        continue;
      }

      // Get angles of ray to get xyz for point
      double yAngle;
      double pAngle;

      if (rangeCount > 1) 
      {
        yAngle = i * yDiff / (rangeCount -1) + minAngle.Radian();
      } 
      else 
      {
        yAngle = minAngle.Radian();
      }

      if (verticalRayCount > 1) 
      {
        pAngle = j * pDiff / (verticalRangeCount -1) + verticalMinAngle.Radian();
      } 
      else 
      {
        pAngle = verticalMinAngle.Radian();
      }

      // Calculate x, y, z, intensity and ring
      *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle); // x
      *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle); // y
      *((float*)(ptr + 8)) = r * sin(pAngle); // z
      *((float*)(ptr + 16)) = intensity; // intensity
      *((uint16_t*)(ptr + 20)) = j; // ring
      ptr += POINT_STEP;
    }
  }

  // Populate message with number of valid points
  msg.point_step = POINT_STEP;
  msg.row_step = ptr - msg.data.data();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step); // Shrink to actual size

  // Publish output
  this->pub_.publish(msg);
}

// Custom callback queue thread
void GazeboRosHorizonLaser::laserQueueThread()
{
  while (this->nh_->ok()) 
  {
    this->laser_queue_.callAvailable(ros::WallDuration(0.01));
  }
}
}