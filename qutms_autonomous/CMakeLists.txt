cmake_minimum_required(VERSION 2.8.3)
project(qutms_autonomous)

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

find_package(gazebo REQUIRED COMPONENTS)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roslaunch
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  pcl_conversions 
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES gazebo_ros_velodyne_laser
  CATKIN_DEPENDS
    gazebo_plugins
    gazebo_ros
    roscpp
    rospy
    std_msgs
    sensor_msgs
    tf
    pcl_conversions 
    pcl_ros
)

roslaunch_add_file_check(launch)
 
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  plugins/include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${GAZEBO_INCLUDE_DIRS}
)

##### Compile plugins and add to library

# qev2_drive_plugin
#add_library(qev2_drive_plugin plugins/qev2_drive_plugin.cc)
#target_link_libraries( qev2_drive_plugin
#${catkin_LIBRARIES} 
#${GAZEBO_LIBRARIES} 
#)

# qev2_velpub plugin
#add_library(qev2_velpub_plugin plugins/qev2_velpub.cc)
#target_link_libraries( qev2_velpub_plugin
#${catkin_LIBRARIES} 
#${GAZEBO_LIBRARIES} 
#)

# horizon lidar plugin
add_library(horizon_lidar_plugin plugins/src/horizon_lidar_plugin.cpp)
target_link_libraries(horizon_lidar_plugin 
${catkin_LIBRARIES} 
${GAZEBO_LIBRARIES} 
RayPlugin
)

# Program to get cone positions from a txt document
add_executable(XML_to_Pose src/xml_to_pose.cpp)
target_link_libraries(XML_to_Pose ${catkin_LIBRARIES})

# Manual control node
add_executable(Joystick_to_Twist src/joy_handle.cpp)
target_link_libraries(Joystick_to_Twist ${catkin_LIBRARIES})

# State tracker
add_executable(AS_State_Machine src/state_machine.cpp)
target_link_libraries(AS_State_Machine ${catkin_LIBRARIES})

# For mapping
add_executable(Curve_Fitting src/spline_fit.cpp)
target_link_libraries(Curve_Fitting ${catkin_LIBRARIES})
