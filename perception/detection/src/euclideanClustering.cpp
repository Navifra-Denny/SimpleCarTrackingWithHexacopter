#include "euclideanClustering.hpp"
#include <pcl/filters/passthrough.h>


EuclideanClustering::EuclideanClustering() :
  preprocessed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
  colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
  m_tfListener(m_tfBuffer),
  is_publish_(true)
// : current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), removed_points_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
//   downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
//   nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), onlyfloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
//   diffnormals_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),  
//   clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),     // PointCloud initialization
//   nofloor_cloud_ptr_Rayground(new sensor_msgs::PointCloud2), onlyfloor_cloud_ptr_Rayground(new sensor_msgs::PointCloud2)
{
    ros_namespace_ = ros::this_node::getNamespace(); 
    if (ros_namespace_.substr(0, 2) == "//") ros_namespace_.erase(ros_namespace_.begin()); 
    
    // Publisher
    _pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/points_cluster", 1);
    _pub_ground_cloud = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/points_ground", 1);
    _pub_centroid = nh.advertise<uav_msgs::Centroids>(ros_namespace_ + "/cluster_centroids",1);

    _pub_noground_cloud = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/points_nogroud",1);
    _pub_clusters_message = nh.advertise<uav_msgs::CloudClusterArray>(ros_namespace_ + "/cloud_clusters",100);
    _pub_detected_objects = nh.advertise<uav_msgs::DetectedObjectArray>(ros_namespace_ + "/objects",1);

    // Check Publisher
    _pub_check = nh.advertise<sensor_msgs::PointCloud2>("a", 1);
    _pub_check_ = nh.advertise<sensor_msgs::PointCloud2>("b", 1);
    _pub_TransformedPoints = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/TransformedPoints", 1); 
    _pub_RemovePointsUpTo = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/RemovePointsUpTo", 1);
    _pub_RemovePointsOutside = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/RemovePointsOutside", 1);
    _pub_DownsampleCloud = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/DownsampleCloud", 1);
    _pub_ClipCloud = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/ClipCloud",1);
    _pub_KeepLanePoints = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/KeepLanePoints", 1);
    _pub_toRayGroundFilter = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/toRayGroundFilter", 1);
    _pub_DoNSegmentation = nh.advertise<sensor_msgs::PointCloud2>(ros_namespace_ + "/DoNSegmentation", 1);
    _pub_timer = nh.advertise<uav_msgs::DetectionTime>(ros_namespace_ + "/detection_timer",1);

    // Subscriber
  //  _sub_velodyne = nh.subscribe("/os1_cloud_node/points", 1, &EuclideanClustering::PointCloudCallback, this);  // TF
    _sub_velodyne = nh.subscribe("/velodyne_points", 1, &EuclideanClustering::PointCloudCallback, this);
  //  _sub_velodyne = nh.subscribe("/velodyne_points", 1, &EuclideanClustering::PointCloudCallback, this);
    _sub_chatter = nh.subscribe("/control/char_pub_node/chatter", 10, &EuclideanClustering::ChatterCallback, this);

    // getParam();
    getParam();
    rayGroundFilter.initParam(
        _general_max_slope,
        _local_max_slope,
        _radial_divider_angle,
        _concentric_divider_distance,
        _min_height_threshold,
        _clipping_height,
        _min_point_distance,
        _reclass_distance_threshold,
        _lidar_frame_id, 
        _sensor_height
    );


    GenerateColors(_colors, 255);
    color_index = 0;
}

EuclideanClustering::~EuclideanClustering()
{

}

void EuclideanClustering::getParam()
{
    nh.getParam("euclideanClustering/lidar_frame_id", _lidar_frame_id);

    // Clustering Param
    nh.getParam("euclideanClustering/cluster_size_min", _cluster_size_min);
    nh.getParam("euclideanClustering/cluster_size_max", _cluster_size_max);
    nh.getParam("euclideanClustering/pose_estimation", _pose_estimation);
    nh.getParam("euclideanClustering/cluster_merge_threshold", _cluster_merge_threshold);
    nh.getParam("euclideanClustering/clustering_distance", _clustering_distance);
    nh.getParam("euclideanClustering/use_gpu", _use_gpu);
    nh.getParam("euclideanClustering/use_multiple_thres", _use_multiple_thres);
    nh.getParam("euclideanClustering/clustering_distances", str_distances);
    nh.getParam("euclideanClustering/clustering_ranges", str_ranges);

    // Remove points upto
    nh.getParam("euclideanClustering/remove_points_upto", _remove_points_upto);
    nh.getParam("euclideanClustering/in_distance", _in_distance);

    // Remove points outside
    nh.getParam("euclideanClustering/remove_points_outside", _remove_points_outside);
    nh.getParam("euclideanClustering/out_distance", _out_distance);

    // Ransac Parameter
    nh.getParam("euclideanClustering/remove_ground_ransac", _remove_ground_ransac);
    nh.getParam("euclideanClustering/in_max_height", _in_max_height);
    nh.getParam("euclideanClustering/in_floor_max_angle", _in_floor_max_angle);
    nh.getParam("euclideanClustering/max_iterations", _max_iterations);

    // Ray Ground Parameter
    nh.getParam("euclideanClustering/remove_ground_rayGroundFilter", _remove_ground_rayGroundFilter);
    nh.getParam("euclideanClustering/general_max_slope", _general_max_slope);
    nh.getParam("euclideanClustering/local_max_slope", _local_max_slope);
    nh.getParam("euclideanClustering/radial_divider_angle", _radial_divider_angle);
    nh.getParam("euclideanClustering/concentric_divider_distance", _concentric_divider_distance);
    nh.getParam("euclideanClustering/min_height_threshold", _min_height_threshold);
    nh.getParam("euclideanClustering/clipping_height", _clipping_height);
    nh.getParam("euclideanClustering/min_point_distance", _min_point_distance);
    nh.getParam("euclideanClustering/reclass_distance_threshold", _reclass_distance_threshold);
    nh.getParam("euclideanClustering/sensor_height", _sensor_height);
    nh.getParam("euclideanClustering/output_frame", _output_frame);

    // Clip cloud Parameter
    nh.getParam("euclideanClustering/clip_min_height", _clip_min_height);
    nh.getParam("euclideanClustering/clip_max_height", _clip_max_height);
    nh.getParam("euclideanClustering/ClipCloud", _ClipCloud);

    // Keep lanes Parameter
    nh.getParam("euclideanClustering/keep_lanes", _keep_lanes);
    nh.getParam("euclideanClustering/keep_lane_left_distance", _keep_lane_left_distance);
    nh.getParam("euclideanClustering/keep_lane_right_distance", _keep_lane_right_distance);

    // Downsample Parameter
    nh.getParam("euclideanClustering/downsample_cloud", _downsample_cloud);
    nh.getParam("euclideanClustering/leaf_size", _leaf_size);

    // Diffnormals Parameter
    nh.getParam("euclideanClustering/use_diffnormals", _use_diffnormals);
}


    /************************************/
    //              Main                //
    /************************************/

void EuclideanClustering::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud)
{
    // _pub_check.publish(*transformed_cloud);

    // _velodyne_header = transformed_cloud->header;
    _velodyne_header = in_sensor_cloud->header;

    /************************************/
    //          Preprocessing           //          
    /************************************/
  
    if(!PreprocessCloud(in_sensor_cloud, preprocessed_cloud_ptr)) ROS_ERROR_STREAM("Fail to preprocess PointCloud");
    // if(!PreprocessCloud(transformed_cloud, preprocessed_cloud_ptr)) ROS_ERROR_STREAM("Fail to preprocess PointCloud");
    // if(!PreprocessCloud(in_sensor_cloud, preprocessed_cloud_ptr)) ROS_ERROR_STREAM("Fail to preprocess PointCloud");
    // PublishCloud(&_pub_check, preprocessed_cloud_ptr);

    /************************************/
    //          Segmentation            //
    /************************************/

    if(!SegmentByDistance(preprocessed_cloud_ptr, colored_clustered_cloud_ptr, centroids, cloud_clusters)) ROS_ERROR_STREAM("Fail to cluster pointcloud");   

    /************************************/
    //              Publish             //
    /************************************/

    else if(!PublishColorCloud(&_pub_cluster_cloud, colored_clustered_cloud_ptr)) ROS_ERROR_STREAM("Fail to publish color cloud");
    centroids.header = _velodyne_header;
    if(!PublishCentroids(&_pub_centroid, centroids)) ROS_ERROR_STREAM("Fail to publish centroids");
    cloud_clusters.header = _velodyne_header;
    if(!PublishCloudClusters(&_pub_clusters_message, cloud_clusters)) ROS_ERROR_STREAM("Fail to publish CloudClusters");
    _pub_timer.publish(detection_time_);
}



bool EuclideanClustering::PreprocessCloud(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passThrough_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2::Ptr nofloor_cloud_ptr_Rayground(new sensor_msgs::PointCloud2);           // rayground filter 거친 pointcloud ptr
    sensor_msgs::PointCloud2::Ptr onlyfloor_cloud_ptr_Rayground(new sensor_msgs::PointCloud2);  
    pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr diffnormals_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // _pub_check_.publish(*in_sensor_cloud);

    ros::Time timer = ros::Time::now();
    ros::Duration eclipse;


    // Downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (_downsample_cloud){
        pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &p : current_sensor_cloud_ptr->points){
          auto distance = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0) + pow(p.z, 2.0));

          if (distance > _in_distance){
            const int MAX_TH = 10.0;
            const int MIN_TH = -10.0;
            if ((p.y < MAX_TH && p.y > MIN_TH) &&
            (p.z < MAX_TH && p.z > MIN_TH) &&
            (p.x < 20.0 && p.x > -20.0)){
              cloud_ptr->points.push_back(p);
            }
          }
        }
        cloud_ptr->header.frame_id = "velodyne";

        if (!DownsampleCloud(cloud_ptr, downsampled_cloud_ptr, _leaf_size)) ROS_ERROR_STREAM("Fail to downsampling");
        PublishCloud(&_pub_DownsampleCloud, downsampled_cloud_ptr);
    }
    else downsampled_cloud_ptr = current_sensor_cloud_ptr;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*downsampled_cloud_ptr, cloud_msg);
    cloud_msg.header = _velodyne_header;

    
    eclipse = ros::Time::now() - timer;
    detection_time_.downsample = eclipse.toSec();
    timer = ros::Time::now();
    

    geometry_msgs::TransformStamped transformStamped;
	  sensor_msgs::PointCloud2::Ptr transformed_cloud(new sensor_msgs::PointCloud2);
	  try{
	  	transformStamped = m_tfBuffer.lookupTransform("map", "velodyne", ros::Time(0));
	  	tf2::doTransform(cloud_msg, *transformed_cloud, transformStamped);
	  }
	  catch (tf2::TransformException &ex) {
	  	ROS_WARN("%s", ex.what());
	  	ros::Duration(1.0).sleep();
	  }
    
    // PublishCloud(&_pub_check, transformed_cloud);
    _pub_check.publish(transformed_cloud);

    eclipse = ros::Time::now() - timer;
    detection_time_.transform = eclipse.toSec();
    timer = ros::Time::now();




    _velodyne_header.frame_id = "map";



    
    // Ground Removal 
    // Choose 1. Rayground 2. Ransac 3. No filtering
    if (_remove_ground_rayGroundFilter){
        rayGroundFilter.initParam(_general_max_slope, _local_max_slope, _radial_divider_angle, _concentric_divider_distance, 
                                  _min_height_threshold, _clipping_height, _min_point_distance, _reclass_distance_threshold, _lidar_frame_id, _sensor_height);  
        if(!rayGroundFilter.removeFloor_rayGroundFilter(in_sensor_cloud, nofloor_cloud_ptr_Rayground, onlyfloor_cloud_ptr_Rayground)) ROS_ERROR_STREAM("Fail to remove floor with Rayground filter");
        _pub_noground_cloud.publish(*nofloor_cloud_ptr_Rayground);
        _pub_ground_cloud.publish(*onlyfloor_cloud_ptr_Rayground);
        pcl::fromROSMsg(*nofloor_cloud_ptr_Rayground, *nofloor_cloud_ptr);
    }
    else if (_remove_ground_ransac){
        // pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
        pcl::fromROSMsg(*transformed_cloud, *current_sensor_cloud_ptr);
        // if(!PassThrough(current_sensor_cloud_ptr, passThrough_cloud_ptr)) ROS_ERROR_STREAM("Fail to passthrough poincloud");
        // if(!RemoveFloorRansac(current_sensor_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr, _in_max_height, _in_floor_max_angle)) ROS_ERROR_STREAM("Fail to remove floor");
        if(!RemoveFloorRansac(current_sensor_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr, _in_max_height, _in_floor_max_angle)) ROS_ERROR_STREAM("Fail to remove floor");
        PublishCloud(&_pub_ground_cloud, onlyfloor_cloud_ptr);
        PublishCloud(&_pub_noground_cloud, nofloor_cloud_ptr);
    }   
    else{
        pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
        nofloor_cloud_ptr = downsampled_cloud_ptr;
    }

    eclipse = ros::Time::now() - timer;
    detection_time_.remove_ground = eclipse.toSec();
    timer = ros::Time::now();
    
    // PublishCloud(&_pub_check, nofloor_cloud_ptr);

    // Remove pointcloud within Threshold
    pcl::PointCloud<pcl::PointXYZ>::Ptr threshold_removed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    ThresholdRemoveCloud(nofloor_cloud_ptr, threshold_removed_cloud_ptr); 

       
    // Downsampling
    // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // if (_downsample_cloud){
    //     if (!DownsampleCloud(threshold_removed_cloud_ptr, downsampled_cloud_ptr, _leaf_size)) ROS_ERROR_STREAM("Fail to downsampling");
    //     PublishCloud(&_pub_DownsampleCloud, downsampled_cloud_ptr);
    // }
    // else downsampled_cloud_ptr = threshold_removed_cloud_ptr; 

    eclipse = ros::Time::now() - timer;
    detection_time_.downsample = eclipse.toSec();
    timer = ros::Time::now();
    
    if(_use_diffnormals) 
    {
      if(!DifferenceNormalsSegmentation(threshold_removed_cloud_ptr, diffnormals_cloud_ptr)) ROS_ERROR_STREAM("Fail to calculate difference normlas");
      PublishCloud(&_pub_DoNSegmentation, diffnormals_cloud_ptr);
    }
    else diffnormals_cloud_ptr = threshold_removed_cloud_ptr;

    eclipse = ros::Time::now() - timer;
    detection_time_.normal_segmentation = eclipse.toSec();
    timer = ros::Time::now();
    
    // *out_cloud_ptr = *diffnormals_cloud_ptr;
    *out_cloud_ptr = *diffnormals_cloud_ptr;

    return true;
}

// bool EuclideanClustering::PassThrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
//                                       pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
// {
//     pcl::PassThrough<pcl::PointXYZ> pass_filter;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);

//     ptr_filtered = in_cloud_ptr;
//     PublishCloud(&_pub_check, ptr_filtered);
//     // pass_filter.setInputCloud(in_cloud_ptr);

//     return true;

// }

bool EuclideanClustering::ThresholdRemoveCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{

    // Remove pointcloud within ROI 
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_upto_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_outside_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (_remove_points_upto){
        if (!RemovePointsUpTo(in_cloud_ptr, removed_points_cloud_upto_ptr, _in_distance)) ROS_ERROR_STREAM("Fail to remove inside PointCloud");
        PublishCloud(&_pub_RemovePointsUpTo, removed_points_cloud_upto_ptr);
    }
    else removed_points_cloud_upto_ptr = in_cloud_ptr;

    if (_remove_points_outside){
        if (!RemovePointsOutside(removed_points_cloud_upto_ptr, removed_points_cloud_outside_ptr, _out_distance)) ROS_ERROR_STREAM("Fail to remove outside Pointcloud");
        PublishCloud(&_pub_RemovePointsOutside, removed_points_cloud_outside_ptr);
    }
    else removed_points_cloud_outside_ptr = removed_points_cloud_upto_ptr;

    // Trim with z axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (_ClipCloud){
       if (!ClipCloud(removed_points_cloud_outside_ptr, clipped_cloud_ptr, _clip_min_height, _clip_max_height)) ROS_ERROR_STREAM("Fail to clip"); 
       PublishCloud(&_pub_ClipCloud, clipped_cloud_ptr);      
    }
    else clipped_cloud_ptr = removed_points_cloud_outside_ptr;

    // Trim with y axis 
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (_keep_lanes){
        if(!KeepLanePoints(clipped_cloud_ptr, inlanes_cloud_ptr, _keep_lane_left_distance, _keep_lane_right_distance)) ROS_ERROR_STREAM("Fail to KeepLanePoints");
        PublishCloud(&_pub_KeepLanePoints, inlanes_cloud_ptr);
    }
    else inlanes_cloud_ptr = clipped_cloud_ptr;

    *out_cloud_ptr = *inlanes_cloud_ptr;

    return true;
}


bool EuclideanClustering::RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].y, 2));
    if (origin_distance > in_distance)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
  if (out_cloud_ptr->points.size() < 1) return false;
  
  return true;
}

bool EuclideanClustering::RemovePointsOutside(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].y, 2));
    if (origin_distance < in_distance)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
  if (out_cloud_ptr->points.size() < 1) return false;
  
  return true;
}

bool EuclideanClustering::DownsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  bool _remove_ground;
  sor.setInputCloud(in_cloud_ptr);
  sor.setLeafSize((float) in_leaf_size, (float) in_leaf_size, (float) in_leaf_size);
  sor.filter(*out_cloud_ptr);

  return true;
}

bool EuclideanClustering::ClipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                                    float in_min_height, float in_max_height)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    if (in_cloud_ptr->points[i].z >= in_min_height && in_cloud_ptr->points[i].z <= in_max_height)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
  return true;
}

bool EuclideanClustering:: KeepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, 
                                          float in_left_lane_threshold, float in_right_lane_threshold)
{
  pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZ current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;

    if (current_point.y > (in_left_lane_threshold) || current_point.y < -1.0 * in_right_lane_threshold)
    {
      far_indices->indices.push_back(i);
    }
  }
  out_cloud_ptr->points.clear();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(far_indices);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);
  
  return true;
}

void EuclideanClustering::PublishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = _velodyne_header;
    in_publisher->publish(cloud_msg);
}

bool EuclideanClustering::RemoveFloorRansac(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, 
                                      float in_max_height,
                                      float in_floor_max_angle)
{
  // pcl::SACSegmentation<pcl::PointXYZ> seg;
  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  // seg.setOptimizeCoefficients(true);
  // seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  // seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setMaxIterations(200);
  // seg.setAxis(Eigen::Vector3f(0, 0, 1));
  // seg.setEpsAngle(in_floor_max_angle);

  // seg.setDistanceThreshold(in_max_height);  // floor distance
  // seg.setOptimizeCoefficients(true);
  // seg.setInputCloud(in_cloud_ptr);
  // seg.segment(*inliers, *coefficients);
  // if (inliers->indices.size() == 0)
  // {
  //   std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  // }

  // // REMOVE THE FLOOR FROM THE CLOUD
  // pcl::ExtractIndices<pcl::PointXYZ> extract;
  // extract.setInputCloud(in_cloud_ptr);
  // extract.setIndices(inliers);
  // extract.setNegative(true);  // true removes the indices, false leaves only the indices
  // extract.filter(*out_nofloor_cloud_ptr);

  // // EXTRACT THE FLOOR FROM THE CLOUD
  // extract.setNegative(false);  // true removes the indices, false leaves only the indices
  // extract.filter(*out_onlyfloor_cloud_ptr);


  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(in_max_height);  // floor distance
  seg.setMaxIterations(_max_iterations);
  seg.setInputCloud(in_cloud_ptr);
  seg.segment(*inliers, *coefficients);

//   seg.setAxis(Eigen::Vector3f(0, 0, 1));
  // seg.setEpsAngle(in_floor_max_angle);

  // seg.setOptimizeCoefficients(true);
  if (inliers->indices.size() == 0)
  {
    // std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // REMOVE THE FLOOR FROM THE CLOUD
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_nofloor_cloud_ptr);

  // EXTRACT THE FLOOR FROM THE CLOUD
  extract.setNegative(false);  // true removes the indices, false leaves only the indices
  extract.filter(*out_onlyfloor_cloud_ptr);

  return true;
}         

bool EuclideanClustering::DifferenceNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
  float small_scale = 0.5;
  float large_scale = 2.0;
  float angle_threshold = 0.5;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree;
  if (in_cloud_ptr->isOrganized())
  {
    tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  } else
  {
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud(in_cloud_ptr);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
  // pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
  normal_estimation.setInputCloud(in_cloud_ptr);
  normal_estimation.setSearchMethod(tree);

  normal_estimation.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max());

  pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

  normal_estimation.setRadiusSearch(small_scale);
  normal_estimation.compute(*normals_small_scale);

  normal_estimation.setRadiusSearch(large_scale);
  normal_estimation.compute(*normals_large_scale);

  pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*in_cloud_ptr, *diffnormals_cloud);

  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> diffnormals_estimator;
  diffnormals_estimator.setInputCloud(in_cloud_ptr);
  diffnormals_estimator.setNormalScaleLarge(normals_large_scale);
  diffnormals_estimator.setNormalScaleSmall(normals_small_scale);

  diffnormals_estimator.initCompute();

  diffnormals_estimator.computeFeature(*diffnormals_cloud);

  pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
    new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, angle_threshold)));
  // Build the filter
  pcl::ConditionalRemoval<pcl::PointNormal> cond_removal;
  cond_removal.setCondition(range_cond);
  cond_removal.setInputCloud(diffnormals_cloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

  // Apply filter
  cond_removal.filter(*diffnormals_cloud_filtered);

  pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*diffnormals_cloud_filtered, *out_cloud_ptr);

  return true;
}

bool EuclideanClustering::SegmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                            uav_msgs::Centroids &in_out_centroids, uav_msgs::CloudClusterArray &in_out_clusters)
{
  // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the
  // entire pc)
  std::vector<ClusterPtr> all_clusters;
  out_cloud_ptr->clear();

  ros::Time timer = ros::Time::now();
  ros::Duration eclipse;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZ current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;
    cloud_ptr->points.push_back(current_point);
  }
  // all_clusters = ClusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance);
  all_clusters = ClusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance);

  eclipse = ros::Time::now() - timer;
  detection_time_.cluster_and_color = eclipse.toSec();
  timer = ros::Time::now();
  
  // Clusters can be merged or checked in here
  //....
  // check for mergable clusters
  std::vector<ClusterPtr> mid_clusters;
  std::vector<ClusterPtr> final_clusters;
  
  if (all_clusters.size() > 0) CheckAllForMerge(all_clusters, mid_clusters, _cluster_merge_threshold);
  else mid_clusters = all_clusters;
  
  // ROS_INFO_STREAM(mid_clusters.size());
  if (mid_clusters.size() > 0) CheckAllForMerge(mid_clusters, final_clusters, _cluster_merge_threshold);
  else final_clusters = mid_clusters;
  // ROS_WARN_STREAM(final_clusters.size());
  
  eclipse = ros::Time::now() - timer;
  detection_time_.cluster_checking = eclipse.toSec();
  timer = ros::Time::now();
  
  // Initialization
  in_out_centroids.points.clear();
  in_out_clusters.clusters.clear();
  // Get final PointCloud to be published
  for (unsigned int i = 0; i < final_clusters.size(); i++)
  {
    if (color_index > 100) color_index = 0;

    // ROS_INFO_STREAM(final_clusters[i]->Id());
    *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());

    jsk_recognition_msgs::BoundingBox bounding_box = final_clusters[i]->GetBoundingBox();
    geometry_msgs::PolygonStamped polygon = final_clusters[i]->GetPolygon();
    // jsk_rviz_plugins::Pictogram pictogram_cluster;
    // pictogram_cluster.header = _velodyne_header;
    // // PICTO
    // pictogram_cluster.mode = pictogram_cluster.STRING_MODE;
    // pictogram_cluster.pose.position.x = final_clusters[i]->GetMaxPoint().x;
    // pictogram_cluster.pose.position.y = final_clusters[i]->GetMaxPoint().y;
    // pictogram_cluster.pose.position.z = final_clusters[i]->GetMaxPoint().z;
    // tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
    // tf::quaternionTFToMsg(quat, pictogram_cluster.pose.orientation);
    // pictogram_cluster.size = 4;
    // std_msgs::ColorRGBA color;
    // color.a = 1;
    // color.r = 1;
    // color.g = 1;
    // color.b = 1;
    // pictogram_cluster.color = color;
    // pictogram_cluster.character = std::to_string(i);
    pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
    geometry_msgs::Point centroid;
    centroid.x = center_point.x;
    centroid.y = center_point.y;
    centroid.z = center_point.z;
    bounding_box.header = _velodyne_header;
    polygon.header = _velodyne_header;

    // CloudCluster msg 에 color 입혀주기 
    std_msgs::ColorRGBA color;
    // int k = final_clusters[i]->Id();
    int k = color_index;
    color.a = 0.8;
    color.r = _colors[k].val[0];
    color.g = _colors[k].val[1];
    color.b = _colors[k].val[2];

    // ROS_WARN_STREAM(color);
    if (final_clusters[i]->IsValid())
    {
      in_out_centroids.points.push_back(centroid);
      uav_msgs::CloudCluster cloud_cluster;
      final_clusters[i]->ToROSMessage(_velodyne_header, cloud_cluster);
      cloud_cluster.color = color;
      cloud_cluster.id = final_clusters[i]->Id();
      in_out_clusters.clusters.push_back(cloud_cluster);  
    }

    color_index ++;
  }
  // ROS_WARN_STREAM("---------");
  final_clusters.size();
  
  eclipse = ros::Time::now() - timer;
  detection_time_.cluster_publish = eclipse.toSec();
  timer = ros::Time::now();
  
  return true;
}

std::vector<ClusterPtr> EuclideanClustering::ClusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                                             uav_msgs::Centroids &in_out_centroids,
                                                             double in_max_cluster_distance)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // create 2d pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);  
   
    //make it flat
    // for (size_t i = 0; i < cloud_2d->points.size(); i++)
    // {
    //   cloud_2d->points[i].z = 0;
    // }
    if (cloud_2d->points.size() > 0) tree->setInputCloud(cloud_2d);

    std::vector<pcl::PointIndices> cluster_indices;

    // perform clustering on 2d cloud
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(in_max_cluster_distance);  //
    ec.setMinClusterSize(_cluster_size_min);
    ec.setMaxClusterSize(_cluster_size_max);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_2d);
    ec.extract(cluster_indices);    
    // use indices on 3d cloud
    // Color clustered points
     unsigned int k = 0;    
     std::vector<ClusterPtr> clusters;
  
    // cluster
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        ClusterPtr cluster(new Cluster());
        cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int) _colors[k].val[0], (int) _colors[k].val[1],
                        (int) _colors[k].val[2], "", _pose_estimation); 
        clusters.push_back(cluster);    
        k++;
    }   
    return clusters;
}

void EuclideanClustering::CheckAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters, float in_merge_threshold)
{
    std::vector<bool> visited_clusters(in_clusters.size(), false);
    std::vector<bool> merged_clusters(in_clusters.size(), false);   
    size_t current_index = 0;
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
        if (!visited_clusters[i])
        {
            visited_clusters[i] = true;
            std::vector<size_t> merge_indices;

            // merge 조건에 맞는 cluster indices 구하는 과정
            if(!CheckClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold)) ROS_ERROR_STREAM("Fail to check cluster merge");
            if(!MergeClusters(in_clusters, out_clusters, merge_indices, current_index++, merged_clusters)) ROS_ERROR_STREAM("Fail to merge clusters");
        }
    }
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
      // check for clusters not merged, add them to the output
      if (!merged_clusters[i])
      {
        out_clusters.push_back(in_clusters[i]);
      }
    } 
    // ROS_ERROR_STREAM(out_clusters.size());

}

bool EuclideanClustering::CheckClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                                            std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                                            double in_merge_threshold)
{
    pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
      if (i != in_cluster_id && !in_out_visited_clusters[i])
      {
        pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
        double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
        if (distance <= in_merge_threshold)
        {
          in_out_visited_clusters[i] = true;
          out_merge_indices.push_back(i);
          // std::cout << "일정 distance이내 " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
          CheckClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
        }
      }
    }
    return true;
}

bool EuclideanClustering::MergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                                        std::vector<size_t> in_merge_indices, const size_t &current_index,
                                        std::vector<bool> &in_out_merged_clusters)
{
    pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
    pcl::PointCloud<pcl::PointXYZ> mono_cloud;
    ClusterPtr merged_cluster(new Cluster());

    // merge pointcloud using merge indices
    for (size_t i = 0; i < in_merge_indices.size(); i++)
    {
      sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
      in_out_merged_clusters[in_merge_indices[i]] = true;
    }

    // Cluster::Setcloud의 arg를 구하기 위한 과정 
    std::vector<int> indices(sum_cloud.points.size(), 0);
    for (size_t i = 0; i < sum_cloud.points.size(); i++)
    {
      indices[i] = i;
    }   
    if (sum_cloud.points.size() > 0)
    {
      pcl::copyPointCloud(sum_cloud, mono_cloud);
      merged_cluster->SetCloud(mono_cloud.makeShared(), indices, _velodyne_header, current_index,
                               (int) _colors[current_index].val[0], (int) _colors[current_index].val[1],
                               (int) _colors[current_index].val[2], "", _pose_estimation);
      out_clusters.push_back(merged_cluster);
    }  
    return true;
}

bool EuclideanClustering::PublishColorCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = _velodyne_header;
    in_publisher->publish(cloud_msg);   
    return true;
}

bool EuclideanClustering::PublishCentroids(const ros::Publisher *in_publisher, const uav_msgs::Centroids &in_centroids)
{
    // std::cout << "number of Clustering: " << in_centroids.points.size() << std::endl;
    in_publisher->publish(in_centroids);

    return true;
}

bool EuclideanClustering::PublishCloudClusters(const ros::Publisher* in_publisher, const uav_msgs::CloudClusterArray& in_clusters)
{
    in_publisher->publish(in_clusters);
    if(!PublishDetectedObjects(in_clusters)) ROS_ERROR_STREAM("Fail to publish detectedobjects");

    return true;
}

// Publish detectedobjects
bool EuclideanClustering::PublishDetectedObjects(const uav_msgs::CloudClusterArray &in_clusters)
{
    uav_msgs::DetectedObjectArray detected_objects;
    detected_objects.header = in_clusters.header;   
    // Initialization
    detected_objects.objects.clear();
    for(size_t i = 0; i < in_clusters.clusters.size(); i++)
    {
      uav_msgs::DetectedObject detected_object;
      detected_object.id = in_clusters.clusters[i].id;
      detected_object.color = in_clusters.clusters[i].color;
      detected_object.header = in_clusters.header;
      detected_object.label = "unknown";
      detected_object.score = 1.;
      detected_object.space_frame = in_clusters.header.frame_id;
      detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
      detected_object.dimensions = in_clusters.clusters[i].dimensions;
      detected_object.pointcloud = in_clusters.clusters[i].cloud;
      detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
      detected_object.valid = true; 
      detected_objects.objects.push_back(detected_object);

    }
    if (is_publish_) _pub_detected_objects.publish(detected_objects); 

    return true;
}




// Generate colors
void EuclideanClustering::GenerateColors(std::vector<Scalar>& colors, size_t count, size_t factor)
{
    ROS_WARN_STREAM("Generating Colors ... ");
    if (count < 1)
      return;   
    colors.resize(count);   
    if (count == 1)
    {
      colors[0] = Scalar(0, 0, 255);  // red
      return;
    }
    if (count == 2)
    {
      colors[0] = Scalar(0, 0, 255);  // red
      colors[1] = Scalar(0, 255, 0);  // green
      return;
    }   
    
    // Generate a set of colors in RGB space. A size of the set is severel times (=factor) larger then
    // the needed count of colors.
    Mat bgr(1, (int)(count * factor), CV_8UC3);
    randu(bgr, 0, 256); 
    // Convert the colors set to Lab space.
    // Distances between colors in this space correspond a human perception.
    Mat lab;
    cvtColor(bgr, lab, cv::COLOR_BGR2Lab);  
    // Subsample colors from the generated set so that
    // to maximize the minimum distances between each other.
    // Douglas-Peucker algorithm is used for this.
    Mat lab_subset;
    DownsamplePoints(lab, lab_subset, count);   
    // Convert subsampled colors back to RGB
    Mat bgr_subset;
    cvtColor(lab_subset, bgr_subset, cv::COLOR_BGR2Lab);    
    CV_Assert(bgr_subset.total() == count);
    for (size_t i = 0; i < count; i++)
    {
      Point3_<uchar> c = bgr_subset.at<Point3_<uchar> >((int)i);
      colors[i] = Scalar(c.x, c.y, c.z);
    }   
    ROS_WARN_STREAM("Generating Colors ... Done ");
}

void EuclideanClustering::DownsamplePoints(const Mat& src, Mat& dst, size_t count)
{
    CV_Assert(count >= 2);
    CV_Assert(src.cols == 1 || src.rows == 1);
    CV_Assert(src.total() >= count);
    CV_Assert(src.type() == CV_8UC3);   
    dst.create(1, (int)count, CV_8UC3);
    // TODO: optimize by exploiting symmetry in the distance matrix
    Mat dists((int)src.total(), (int)src.total(), CV_32FC1, Scalar(0));
    if (dists.empty())
      std::cerr << "Such big matrix cann't be created." << std::endl;

    for (int i = 0; i < dists.rows; i++)
    {
      for (int j = i; j < dists.cols; j++)
      {
        float dist = (float)norm(src.at<Point3_<uchar> >(i) - src.at<Point3_<uchar> >(j));
        dists.at<float>(j, i) = dists.at<float>(i, j) = dist;
      }
    }

    double maxVal;
    Point maxLoc;
    minMaxLoc(dists, 0, &maxVal, 0, &maxLoc);   
    dst.at<Point3_<uchar> >(0) = src.at<Point3_<uchar> >(maxLoc.x);
    dst.at<Point3_<uchar> >(1) = src.at<Point3_<uchar> >(maxLoc.y); 
    Mat activedDists(0, dists.cols, dists.type());
    Mat candidatePointsMask(1, dists.cols, CV_8UC1, Scalar(255));
    activedDists.push_back(dists.row(maxLoc.y));
    candidatePointsMask.at<uchar>(0, maxLoc.y) = 0; 

    for (size_t i = 2; i < count; i++)
    {
        activedDists.push_back(dists.row(maxLoc.x));
        candidatePointsMask.at<uchar>(0, maxLoc.x) = 0;   
        Mat minDists;
        reduce(activedDists, minDists, 0, CV_REDUCE_MIN);
        minMaxLoc(minDists, 0, &maxVal, 0, &maxLoc, candidatePointsMask);
        dst.at<Point3_<uchar> >((int)i) = src.at<Point3_<uchar> >(maxLoc.x);
    }
}

void EuclideanClustering::ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr)
{
    if (chat_ptr->msg == "d") is_publish_ = true;
    else if (chat_ptr->msg == "m") is_publish_ = false;
}