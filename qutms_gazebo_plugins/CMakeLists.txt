cmake_minimum_required(VERSION 3.5)
project(qutms_gazebo_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        gazebo_msgs
        roscpp
        nodelet
        geometry_msgs
        nav_msgs
        ackermann_msgs
        tf2_ros
        tf
        )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

catkin_package(
        CATKIN_DEPENDS
        gazebo_msgs
        roscpp
        nodelet
        geometry_msgs
        nav_msgs
        ackermann_msgs
        tf2_ros
        tf
)

add_subdirectory(gazebo_race_car_model)
