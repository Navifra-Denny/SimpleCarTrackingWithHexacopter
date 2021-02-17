#include "ros/ros.h"
#include "control/visualize_control.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "visualize_control_node");
    VisualizeControl visualize_control;

    ros::spin();

    return 0;
} 