#include "ros/ros.h"
#include "visualization/visualize_control.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "visualization_control_node");
    VisualizeControl visualize_control;

    ros::spin();

    return 0;
} 