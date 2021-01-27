#include "ros/ros.h"
// #include "airsim_ros_wrapper.h"
#include "generate_waypoints.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "generate_waypoints_node");
    GenerateWaypoints gen_waypoints;

    ros::spin();

    return 0;
} 