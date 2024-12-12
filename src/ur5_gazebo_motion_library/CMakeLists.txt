cmake_minimum_required(VERSION 3.0.2)
project(ur5_gazebo_motion_library)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kdl_parser_py
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)


###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
## Declare package settings for dependent projects
catkin_package(
#  INCLUDE_DIRS include             # Header files for this package
#  LIBRARIES ur5_gazebo_motion_library # Libraries exported by this package
#  CATKIN_DEPENDS geometry_msgs kdl_parser_py roscpp rospy sensor_msgs std_msgs trajectory_msgs # Required catkin packages
#  DEPENDS system_lib              # Non-catkin system dependencies
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/ur5_joint_motion_API.py
  scripts/ur5_linear_motion_API.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
