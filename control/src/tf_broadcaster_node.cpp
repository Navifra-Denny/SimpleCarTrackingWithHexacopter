#include "ros/ros.h"
#include "control/tf_broadcaster.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "tf_broadcaster_node");
    TfBroadcaster tf_broadcaster;

    ros::spin();

    return 0;
} 