#include <ros/ros.h>
#include "tracking/kfTrackerCore.hpp"

int main(int argc, char **argv){
    
    ros::init(argc, argv, "kfTracker");
    Tracker tracker;

    ros::spin();
    return 0;
}