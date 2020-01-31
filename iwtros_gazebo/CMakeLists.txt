cmake_minimum_required(VERSION 2.8.3)
project(iwtros_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  sensor_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  laser_geometry
  pcl_ros
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    conveyor_spawner
    urdf_creator
  CATKIN_DEPENDS 
    cmake_modules 
    gazebo_msgs 
    gazebo_plugins
    gazebo_ros 
    geometry_msgs
    sensor_msgs 
    message_runtime 
    roscpp 
    rospy 
    std_msgs 
    std_srvs 
    tf
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

