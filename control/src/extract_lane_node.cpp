#include "ros/ros.h"
#include "control/extract_lane.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "extract_lane_node");
    ExtractLane extract_lane_node;

    ros::spin();

    return 0;
} 