cmake_minimum_required(VERSION 3.0.2)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  nav_msgs
  uav_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs uav_msgs nav_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${uav_msgs_INCLUDE_DIRS}
)

## generate_waypoints_node
add_executable(generate_waypoints_node src/generate_waypoints_node.cpp src/generate_waypoints.cpp src/utils.cpp)
target_link_libraries(generate_waypoints_node ${catkin_LIBRARIES})

add_executable(mavros_offb_node src/mavros_offb_node.cpp src/mavros_offb.cpp src/utils.cpp)
target_link_libraries(mavros_offb_node ${catkin_LIBRARIES})

add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp src/tf_broadcaster.cpp src/utils.cpp)
target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})

add_executable(visualize_control_node src/visualize_control_node.cpp src/visualize_control.cpp src/utils.cpp)
target_link_libraries(visualize_control_node ${catkin_LIBRARIES})

add_executable(checker_node src/checker_node.cpp)
target_link_libraries(checker_node ${catkin_LIBRARIES})

add_executable(evaluation_node src/evaluation_node.cpp src/utils.cpp)
target_link_libraries(evaluation_node ${catkin_LIBRARIES})

add_executable(char_pub_node src/char_pub_node.cpp)
target_link_libraries(char_pub_node ${catkin_LIBRARIES})