#ifndef __KFTRACKCORE_HPP__
#define __KFTRACKCORE_HPP__

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "tracking/SimpleTracker.hpp"
#include "uav_msgs/DetectedObjectArray.h"
#include "uav_msgs/CloudClusterArray.h"

namespace tracking
{
class Point
{
public:
    double x;
    double y;
    double z;
    double a;
    double cost;

    Point()
    {
        x = 0;
        y = 0;
        z = 0;
        a = 0;
        cost = 0;
    }

    Point(const double& x, const double& y, const double& z, const double& a)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->a = a;
        this->cost = 0;
    }
   
};
}


class DetectedObject
{
public:
    int id;
    std::string label;
    tracking::Point center;
    tracking::Point predicted_center;
    std::vector<tracking::Point> centers_list;
    std::vector<tracking::Point> contour;
    double w;
    double l;
    double h;
    double actual_yaw;

    DetectedObject()
    {
        id = 0;
        w = 0;
        l = 0;
        h = 0;
        actual_yaw = 0; 
    }
};


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
    int trackingType;
 
    PerceptionParams()
    {
        VehicleWidth = 0;
        VehicleLength= 0;
        DetectionRadius = 0;
        MinObjSize = 0;
        MaxObjSize = 0;
        nQuarters = 0;
        PolygonRes = 0;
        trackingType = SIMPLE_TRACKER;

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