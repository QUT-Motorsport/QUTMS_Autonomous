cmake_minimum_required(VERSION 2.8.3)
project(qutms_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  qutms_description
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
)

catkin_package()

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
