cmake_minimum_required(VERSION 3.0.2)
project(ur5_sim_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add executable and link it to ROS libraries
add_executable(sine_wave_control src/sine_wave_control.cpp)
target_link_libraries(sine_wave_control ${catkin_LIBRARIES})
