cmake_minimum_required(VERSION 2.8.3)
project(iwtros_goal)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  tf
  tf2
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES iwtros_goal
  CATKIN_DEPENDS actionlib actionlib_msgs control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools roscpp rospy sensor_msgs tf tf2
#  DEPENDS system_lib
)

catkin_python_setup()

catkin_install_python(PROGRAMS 
  script/iwtros_goal/picknplace_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pnp_demo_node 
  src/picknplace_main.cpp
)
target_link_libraries(pnp_demo_node
  ${catkin_LIBRARIES}
)
