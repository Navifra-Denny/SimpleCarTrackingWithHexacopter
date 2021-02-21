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
#include "uav_msgs/Offset.h"

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
    ros::Publisher offset_pub = nh.advertise<uav_msgs::Offset>("/control/char_pub_node/offset", 1000);

    ros::Rate loop_rate(10);

    bool is_first = true;
    while (ros::ok())
    {
        if (is_first){
            is_first = false;
            ROS_INFO_STREAM("/********************************************/");
            ROS_INFO_STREAM("/** command: offboard|manual|set_origin|   **/");
            ROS_INFO_STREAM("/**          global|local|global_z|local_z **/");
            ROS_INFO_STREAM("/**          debugging|rc                  **/");
            ROS_INFO_STREAM("/********************************************/");
        }

        std::string inputString;
        std::cout << "[command] ";
        std::getline(std::cin, inputString);
        std_msgs::String msg;


        if((inputString.compare("offboard") == 0) || 
            (inputString.compare("manual") == 0) ||
            (inputString.compare("set_origin") == 0) ||
            (inputString.compare("global") == 0) ||
            (inputString.compare("local") == 0) ||
            (inputString.compare("debugging") == 0) ||
            (inputString.compare("rc") == 0))
        {
            std::cout << ("[input] " + inputString) << std::endl;
            msg.data = inputString;
            char_pub.publish(msg);
        }
        else if((inputString.compare("global_z") == 0) || 
                (inputString.compare("local_z") == 0))
        {
            bool is_error = true;
            while (is_error){                    
                std::string inputNumString;
                std::cout << "[value] ";
                std::getline(std::cin, inputNumString);

                if (IsNum(inputNumString)){
                    float height = stof(inputNumString);

                    uav_msgs::Offset offset;
                    geographic_msgs::GeoPoint geopoint;
                    geometry_msgs::Point point;
                    bool is_global;
                    if ((inputString.compare("global_z") == 0)){
                        std::cout << ("[input] global altitude offset: " + inputNumString) << std::endl;
                        is_global = true;
                        geopoint.altitude = height;
                    }
                    else {
                        std::cout << ("[input] local z offset: " + inputNumString) << std::endl;
                        is_global = false;
                        point.z = height;
                    }
                    
                    offset.is_global = is_global;
                    offset.geopoint = geopoint;
                    offset.point = point;

                    offset_pub.publish(offset);
                    is_error = false;
                }
                else {
                    ROS_ERROR_STREAM("Please input number");
                    is_error = true;
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 