#include <iostream>
#include <vector>
#include <sstream>
#include <string.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

bool IsNumber(const std::string& str)
{   
    size_t size = str.size();

    if(size == 0) return false;

    for(char const &c : str)
    {
        if(c < '0' || c > '9') 
        return false;
    }
    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "targetIdPublihser");
    ros::NodeHandle nh;

    // target id publisher
    ros::Publisher target_id_pub = nh.advertise<std_msgs::String>("/tracking/target_id_msg", 1);

    bool is_first = true;
    while(ros::ok())
    {
        if (is_first){
            is_first = false;
            std::cerr << "Please, enter [target id]" << std::endl;
        }

        std::string inputString;
        std_msgs::String msg;
        std::cerr << "[target id] ";
        std::getline(std::cin, inputString);

        if(!IsNumber(inputString)) ROS_ERROR_STREAM("You entered the wrong Target id");
        else 
        {
            std::cerr << "You selected target id: " << inputString << std::endl;

            msg.data = inputString;
            target_id_pub.publish(msg);

        }
        

    }

}