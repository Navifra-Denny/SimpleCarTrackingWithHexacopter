/**********************************************************************************************************
 * 3D LiDAR KF-GNN based Multi Object Tracker
 * Date : 2021.02.12
 * Authors : myu6715@gmail.com
 * Requirements : EuclideanClustering node
 * Input : uav_msgs::DetectedObjectArray
 * Output : uav_msgs::DetectedObjectArray
 **********************************************************************************************************/

#include "tracking/KFTracker.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kfTracker");
    KFTracker kfTracker;
    kfTracker.run();

    ros::spin();
    return 0;
}