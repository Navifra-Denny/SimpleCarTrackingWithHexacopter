#include "ros/ros.h"
#include "control/offb.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "offb_node");
    Offboard offboard;

    ros::spin();

    return 0;
} 
