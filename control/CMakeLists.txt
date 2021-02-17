cmake_minimum_required(VERSION 3.0.2)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
set(CONTROL_ROOT "$ENV{HOME}/catkin_ws/src/uav_object_tracking/control")

add_subdirectory("${CONTROL_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${CONTROL_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${CONTROL_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(RPC_LIB_INCLUDES " ${CONTROL_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

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
  ${CONTROL_ROOT}/AirLib/deps/eigen3
  ${CONTROL_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${CONTROL_ROOT}/MavLinkCom/include
  ${CONTROL_ROOT}/MavLinkCom/common_utils
  ${Boost_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS}
  ${uav_msgs_INCLUDE_DIRS}
)

## control_node
# add_executable(airsim_offb_node src/airsim_offb_node.cpp src/airsim_offb.cpp src/utils.cpp)
# target_link_libraries(airsim_offb_node ${catkin_LIBRARIES} AirLib)

## generate_waypoints_node
add_executable(generate_waypoints_node src/generate_waypoints_node.cpp src/generate_waypoints.cpp src/utils.cpp)
target_link_libraries(generate_waypoints_node ${catkin_LIBRARIES})

## extract_lane_node add_executable(display_node src/display_node.cpp)
# add_executable(extract_lane_node src/extract_lane_node.cpp src/extract_lane.cpp src/utils.cpp)
# target_link_libraries(extract_lane_node ${catkin_LIBRARIES})

add_executable(mavros_offb_node src/mavros_offb_node.cpp src/mavros_offb.cpp src/utils.cpp)
target_link_libraries(mavros_offb_node ${catkin_LIBRARIES})

# add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp src/tf_broadcaster.cpp src/utils.cpp)
# target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})

add_executable(visualize_control_node src/visualize_control_node.cpp src/visualize_control.cpp)
target_link_libraries(visualize_control_node ${catkin_LIBRARIES})

add_executable(checker_node src/checker_node.cpp)
target_link_libraries(checker_node ${catkin_LIBRARIES})

add_executable(char_pub_node src/char_pub_node.cpp)
target_link_libraries(char_pub_node ${catkin_LIBRARIES})