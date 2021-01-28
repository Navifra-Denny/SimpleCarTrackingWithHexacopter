cmake_minimum_required(VERSION 3.0.2)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  nav_msgs
  uav_msgs
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

## control_node
add_executable(${PROJECT_NAME}_node src/offb_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

## generate_waypoints_node
add_executable(generate_waypoints_node src/generate_waypoints_node.cpp src/generate_waypoints.cpp)
target_link_libraries(generate_waypoints_node ${catkin_LIBRARIES})

## display_node add_executable(display_node src/display_node.cpp)
add_executable(display_node src/display_node.cpp)
target_link_libraries(display_node ${catkin_LIBRARIES})

## extract_lane_node add_executable(display_node src/display_node.cpp)
add_executable(extract_lane_node src/extract_lane_node.cpp src/extract_lane.cpp)
target_link_libraries(extract_lane_node ${catkin_LIBRARIES})
