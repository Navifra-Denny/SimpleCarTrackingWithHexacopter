#ifndef __KFTRACKCORE_HPP__
#define __KFTRACKCORE_HPP__

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "TypeInfo.hpp"
#include "tracking/SimpleTracker.hpp"
#include "uav_msgs/DetectedObjectArray.h"
#include "uav_msgs/CloudClusterArray.h"


class PerceptionParams
{
public:
    double VehicleWidth;
    double VehicleLength;
    double DetectionRadius;
    double MinObjSize;
    double MaxObjSize;
    double nQuarters;
    double PolygonRes;
 
    PerceptionParams()
    {
        VehicleWidth = 0;
        VehicleLength= 0;
        DetectionRadius = 0;
        MinObjSize = 0;
        MaxObjSize = 0;
        nQuarters = 0;
        PolygonRes = 0;
    }
};


class Tracker
{
private:
    std::vector<DetectedObject> m_OriginalClusters;
    PerceptionParams m_Params;
    SimpleTracker m_ObstacleTracking;

    int m_nOriginalPoints;
    int m_nContourPoints;
    double m_FilteringTime;
    double m_PolyEstimationTime;
    double m_tracking_time;
    double m_dt;
    struct timespec m_loop_timer;

    // ROS 
    ros::NodeHandle nh;
    // Define Subscriber
    ros::Subscriber _sub_cloud_clusters;
    // Define Publisher

public:
    Tracker();
    ~Tracker();

private:
    // Callback function for Subscriber 
    void callbackGetCloudClusters(const uav_msgs::CloudClusterArrayConstPtr &msg);
    void ImportCloudClusters(const uav_msgs::CloudClusterArrayConstPtr& msg, std::vector<DetectedObject>& originalClusters);

    // Helper function 
    void ReadNodeParams(); 
    bool IsCar(const DetectedObject& obj);


};


#endif