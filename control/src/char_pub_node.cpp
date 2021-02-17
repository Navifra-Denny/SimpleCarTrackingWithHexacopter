/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "ros/ros.h"
#include <ros/spinner.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <iostream>
#include <string.h>

bool IsNum(const std::string& str) {
    size_t size = str.size();
    
    if (size == 0) return 0;

    for (char const &c : str) {
        if (c == '.' || c == '-' || c == '+') continue;
        if (c < '0' || c > '9') return false;
    }

    return true; // 그밖의 경우는 숫자임
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "char_pub_node");
    ros::NodeHandle nh;

    ros::Publisher char_pub = nh.advertise<std_msgs::String>("/control/char_pub_node/chatter", 1000);
    ros::Publisher z_target_pub = nh.advertise<geometry_msgs::Point>("/control/char_pub_node/z_target", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        std::string inputString;
        std::getline(std::cin, inputString);
        std_msgs::String msg;

        if((inputString.compare("offboard") == 0) || 
            (inputString.compare("manual") == 0))
        {
            msg.data = inputString;
            char_pub.publish(msg);
        }
        else if (IsNum(inputString)){
            float z = stof(inputString);
            geometry_msgs::Point point;
            point.z = z;
            z_target_pub.publish(point);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 