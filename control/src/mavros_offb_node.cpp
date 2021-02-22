/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "ros/ros.h"
#include "control/mavros_offb.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "mavros_offb_node");
    mavros::Offboard offboard;

    ros::spin();

    return 0;
} 