cmake_minimum_required(VERSION 2.8.3)
project(qutms_autonomous)

add_compile_options(-std=c++11)

find_package(gazebo REQUIRED COMPONENTS)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roslaunch
  roscpp
  rospy
  std_msgs
  pcl_conversions 
  pcl_ros
  velodyne_gazebo_plugins
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
    gazebo_plugins
    gazebo_ros
    roscpp
    rospy
    std_msgs
    pcl_conversions 
    pcl_ros
    velodyne_gazebo_plugins
#  DEPENDS system_lib
)

roslaunch_add_file_check(launch)

install(
  DIRECTORY config include launch nodes plugins src urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

###########
## Build ##
###########
 
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
 
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIR} 
  ${GAZEBO_INCLUDE_DIRS}
)

##### Compile plugins and add to library

# qev2_drive_plugin
add_library(qev2_drive_plugin plugins/qev2_drive_plugin.cc)
target_link_libraries( qev2_drive_plugin
${catkin_LIBRARIES} 
${GAZEBO_LIBRARIES} 
)

# qev2_velpub plugin
add_library(qev2_velpub_plugin plugins/qev2_velpub.cc)
target_link_libraries( qev2_velpub_plugin
${catkin_LIBRARIES} 
${GAZEBO_LIBRARIES} 
)

# Node to transform Laserscan to PointCloud2
# add_executable(ls_to_pc2_node nodes/ls_to_pc2.cpp)
# target_link_libraries(ls_to_pc2_node ${catkin_LIBRARIES})

# add_executable(XML_to_Grid xml_to_grid.cpp)
# target_link_libraries(XML_to_Grid ${catkin_LIBRARIES})

# add_executable(
