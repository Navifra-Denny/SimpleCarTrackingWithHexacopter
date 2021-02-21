#include "ros/ros.h"
#include "control/airsim_offb.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "airsim_offb_node");
    airsim::Offboard offboard;

    ros::spin();

    return 0;
} 
