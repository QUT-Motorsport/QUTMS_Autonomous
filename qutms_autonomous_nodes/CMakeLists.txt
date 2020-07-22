cmake_minimum_required(VERSION 2.8.3)
project(qutms_autonomous_nodes)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable debugging
set( CMAKE_BUILD_TYPE Debug )

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roslaunch
  geometry_msgs
  image_transport
  cv_bridge
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions 
  pcl_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS 
    geometry_msgs 
    roscpp 
    std_msgs
    sensor_msgs
    visualization_msgs
    pcl_conversions 
    pcl_ros
)

roslaunch_add_file_check(launch)

add_executable(lidar_clustering_node src/lidar_clustering.cpp)
target_link_libraries(lidar_clustering_node ${catkin_LIBRARIES})

add_executable(XML_to_Pose src/xml_to_pose.cpp)
target_link_libraries(XML_to_Pose ${catkin_LIBRARIES})

add_executable(AS_State_Machine src/state_machine.cpp)
target_link_libraries(AS_State_Machine ${catkin_LIBRARIES})

add_executable(Curve_Fitting src/spline_fit.cpp)
target_link_libraries(Curve_Fitting ${catkin_LIBRARIES})

add_executable(GPS_Publisher src/gps_pub.cpp)
target_link_libraries(GPS_Publisher ${catkin_LIBRARIES})

add_executable(IMU_Publisher src/imu_pub.cpp)
target_link_libraries(IMU_Publisher ${catkin_LIBRARIES})

add_executable(AKF_Node src/kalman_filter.cpp)
target_link_libraries(AKF_Node ${catkin_LIBRARIES})

add_executable(Image_Pipeline src/image_pipeline.cpp)
target_link_libraries(Image_Pipeline ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
