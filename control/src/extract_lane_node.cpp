#include "ros/ros.h"
#include "off_board_control/extract_lane_node.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "extract_lane_node");
    ExtractLane extract_lane_node;

    ros::spin();

    return 0;
} 