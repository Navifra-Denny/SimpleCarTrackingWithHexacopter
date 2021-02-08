#include "tracking/kfTrackerCore.hpp"
#include "tracking/PolygonGenerator.hpp"

#define __APP_NAME__ "kfTracker"

Tracker::Tracker()
{   
    ReadNodeParams();
    _sub_cloud_clusters = nh.subscribe("/detection/cloud_clusters", 1, &Tracker::callbackGetCloudClusters, this);
}

Tracker::~Tracker()
{  
}

void Tracker::ReadNodeParams()
{
    // PerceptionParams
    nh.getParam("kfTracker/vehicle_width", m_Params.VehicleWidth);
    ROS_INFO("[%s] vehicle_width: %f", __APP_NAME__, m_Params.VehicleWidth);
    nh.getParam("kfTracker/vehicle_length", m_Params.VehicleLength);
    ROS_INFO("[%s] vehicle_length: %f", __APP_NAME__, m_Params.VehicleLength);
    nh.getParam("kfTracker/min_object_size", m_Params.MinObjSize);
    ROS_INFO("[%s] min_object_size: %f", __APP_NAME__, m_Params.MinObjSize);
    nh.getParam("kfTracker/max_object_size", m_Params.MaxObjSize);
    ROS_INFO("[%s] max_object_size: %f", __APP_NAME__, m_Params.MaxObjSize);
    nh.getParam("kfTracker/polygon_quarters", m_Params.nQuarters);
    ROS_INFO("[%s] polygon_quarters: %f", __APP_NAME__, m_Params.nQuarters);
    nh.getParam("kfTracker/polygon_resolution",  m_Params.PolygonRes);
    ROS_INFO("[%s] polygon_resolution: %f", __APP_NAME__, m_Params.PolygonRes);
    nh.getParam("kfTracker/horizonDistance",  m_Params.DetectionRadius);
    ROS_INFO("[%s] DetectionRadius: %f", __APP_NAME__, m_Params.DetectionRadius);
    
    // SimpleTracker Params
    nh.getParam("kfTracker/max_association_distance", m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE);
    ROS_INFO("[%s] max_association_distance: %f", __APP_NAME__, m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE);
    nh.getParam("kfTracker/max_association_size_diff",  m_ObstacleTracking.m_MAX_ASSOCIATION_SIZE_DIFF);
    ROS_INFO("[%s] max_association_size_diff: %f", __APP_NAME__, m_ObstacleTracking.m_MAX_ASSOCIATION_SIZE_DIFF);
    
}

void Tracker::callbackGetCloudClusters(const uav_msgs::CloudClusterArrayConstPtr &msg)
{
    ROS_ERROR_STREAM(msg->clusters.size());
    ImportCloudClusters(msg, m_OriginalClusters);
}

void Tracker::ImportCloudClusters(const uav_msgs::CloudClusterArrayConstPtr& msg, std::vector<DetectedObject>& originalClusters)
{
    originalClusters.clear();
    m_nOriginalPoints = 0;
    m_nContourPoints = 0;
    m_FilteringTime = 0;
    m_PolyEstimationTime = 0;
    struct timespec filter_time;
    struct timespec poly_est_time;

    DetectedObject obj;
    tracking::Point avg_center;
    PolygonGenerator polyGen(m_Params.nQuarters);   // nQuarters = 16
    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    for (unsigned int i = 0; i < msg->clusters.size(); i++)
    {
        obj.center.x = msg->clusters.at(i).centroid_point.point.x;
        obj.center.y = msg->clusters.at(i).centroid_point.point.y;
        obj.center.z = msg->clusters.at(i).centroid_point.point.z;
        obj.center.a = msg->clusters.at(i).estimated_angle;
        
        obj.actual_yaw = msg->clusters.at(i).estimated_angle;
        
        obj.w = msg->clusters.at(i).dimensions.x;
        obj.l = msg->clusters.at(i).dimensions.y;
        obj.h = msg->clusters.at(i).dimensions.z;
        
        // CAR Filtering 
        if(!IsCar(obj)){
            ROS_ERROR_STREAM("Is not a car");               
            continue;
        }

        obj.id = msg->clusters.at(i).id;
        obj.label = msg->clusters.at(i).label;

        point_cloud.clear();
        pcl::fromROSMsg(msg->clusters.at(i).cloud, point_cloud);

        obj.contour = polyGen.EstimateClusterPolygon(point_cloud, obj.center, avg_center, m_Params.DetectionRadius);
        
        m_nOriginalPoints += point_cloud.points.size();
        m_nContourPoints += obj.contour.size();

        ROS_INFO_STREAM(m_nOriginalPoints);
        ROS_INFO_STREAM(m_nContourPoints);
        
        originalClusters.push_back(obj);
    }

}


bool Tracker::IsCar(const DetectedObject& obj)
{
    double object_size = hypot(obj.w, obj.l);
    // ROS_INFO_STREAM(obj.w);
    // ROS_INFO_STREAM(obj.l);


    if(object_size < m_Params.MinObjSize || object_size > m_Params.MaxObjSize)
        return false;    

    return true;

}