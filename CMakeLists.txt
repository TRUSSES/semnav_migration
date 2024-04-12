# MIT License (modified)

# Copyright (c) 2020 The Trustees of the University of Pennsylvania
# Authors:
# Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this **file** (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.5)
project(semnav)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

# message_generation dependency broken
find_package(object_pose_interface_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)

# Define targets
add_library(reactive_planner
  src/libraries/polygeom_lib.cpp
  src/libraries/reactive_planner_lib.cpp
)

ament_target_dependencies(reactive_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_filters
  object_pose_interface_msgs
  tf2_ros
)

# Add executables
add_executable(navigation 
  src/navigation.cpp
)

add_executable(navigation_semantic 
  src/navigation_semantic.cpp
)

add_executable(navigation_humans 
  src/navigation_humans.cpp
)

add_executable(human_following 
  src/human_following.cpp
)

add_executable(human_following_signal 
  src/human_following_signal.cpp
)

add_executable(human_following_fallen
  src/human_following_fallen.cpp
)

add_executable(map_debug 
  src/map_debug.cpp
)

add_executable(fake_lidar_publisher 
  src/fake_lidar_publisher.cpp
)

add_executable(fake_odometry_publisher 
  src/fake_odometry_publisher.cpp
)

# Link libraries
target_link_libraries(navigation reactive_planner)
target_link_libraries(navigation_semantic reactive_planner)
target_link_libraries(navigation_humans reactive_planner)
target_link_libraries(human_following reactive_planner)
target_link_libraries(human_following_signal reactive_planner)
target_link_libraries(human_following_fallen reactive_planner)
target_link_libraries(map_debug reactive_planner)
target_link_libraries(fake_lidar_publisher reactive_planner)
target_link_libraries(fake_odometry_publisher reactive_planner)

# Install
install(TARGETS
  reactive_planner
  navigation
  navigation_semantic
  navigation_humans
  human_following
  human_following_signal
  human_following_fallen
  map_debug
  fake_lidar_publisher
  fake_odometry_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()