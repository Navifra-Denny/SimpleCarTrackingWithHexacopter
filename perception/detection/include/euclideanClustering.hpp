
#ifndef __EUCLIDEANCLUSTERING_HPP__
#define __EUCLIDEANCLUSTERING_HPP__

#include "dataType.hpp"
#include "rayGroundFilter.hpp"
#include "cluster.hpp"

using namespace cv;

class EuclideanClustering
{
public:
    // Constructor
    explicit EuclideanClustering();
    virtual ~EuclideanClustering();

private:
    // Node Handler
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber _sub_velodyne;

    // Publisher
    ros::Publisher _pub_cluster_cloud;
    ros::Publisher _pub_ground_cloud;
    ros::Publisher _pub_centroid;
    ros::Publisher _pub_clusters_message;
    ros::Publisher _pub_noground_cloud;
    ros::Publisher _pub_detected_objects;

    // Check Publisher
    ros::Publisher _pub_RemovePointsUpTo;
    ros::Publisher _pub_DownsampleCloud;
    ros::Publisher _pub_ClipCloud;
    ros::Publisher _pub_KeepLanePoints;
    ros::Publisher _pub_toRayGroundFilter; 
    ros::Publisher _pub_DoNSegmentation;
    ros::Publisher _pub_check;

    // velodyne header 
    std_msgs::Header _velodyne_header;

    // Param
    std::string ros_namespace_;   
    bool _remove_ground_ransac;
    bool _remove_ground_rayGroundFilter;
    bool _downsample_cloud;
    double _leaf_size;
    int _cluster_size_min;
    int _cluster_size_max;
    bool _use_diffnormals;
    bool _pose_estimation;
    bool _keep_lanes;
    double _keep_lane_left_distance;
    double _keep_lane_right_distance;
    bool _ClipCloud;
    double _clip_min_height;
    double _clip_max_height;
    double _remove_points_upto;
    double _clustering_distance;
    double _cluster_merge_threshold;
    bool _use_gpu;
    bool _use_multiple_thres;
    std::string _output_frame;
    float _in_max_height;
    float _in_floor_max_angle;
   


    // Ray Ground Param
    double _general_max_slope;
    double _local_max_slope;
    double _radial_divider_angle;
    double _concentric_divider_distance;
    double _min_height_threshold;
    double _clipping_height;
    double _min_point_distance;
    double _reclass_distance_threshold;
    
  
    std::string str_distances;
    std::string str_ranges;

    // PointCloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessed_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr diffnormals_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr;




    // RayGroundFilter class object 
    RayGroundFilter rayGroundFilter;
    // sensor_msgs::PointCloud2::Ptr nofloor_cloud_ptr_Rayground;           // rayground filter 거친 pointcloud ptr
    // sensor_msgs::PointCloud2::Ptr onlyfloor_cloud_ptr_Rayground;  

    // Cluster msg
    uav_msgs::Centroids centroids;
    uav_msgs::CloudClusterArray cloud_clusters;

    // Generate colors
    std::vector<cv::Scalar> _colors;

private: 
    
    /************************************/
    //              Main                //
    /************************************/
    void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud);

    /************************************/
    //          Preprocessing           //          
    /************************************/

    bool PreprocessCloud(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);

    bool ThresholdRemoveCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);

    bool RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance);
    bool DownsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size = 0.2);
    bool ClipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height = -1.3, float in_max_height = 0.5);
    bool KeepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold = 1.5, float in_right_lane_threshold = 1.5);
    bool RemoveFloorRansac(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, float in_max_height = 0.2, float in_floor_max_angle = 0.1);
    bool DifferenceNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);



    /************************************/
    //          Segmentation            //
    /************************************/
 
    bool SegmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr, 
                           uav_msgs::Centroids &in_out_centroids, uav_msgs::CloudClusterArray &in_out_clusters);
    
    std::vector<ClusterPtr> ClusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                            uav_msgs::Centroids &in_out_centroids, double in_max_cluster_distance = 0.5);

    void CheckAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters, float in_merge_threshold);

    bool CheckClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                       std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                       double in_merge_threshold);

    bool MergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t &current_index,
                   std::vector<bool> &in_out_merged_clusters);

 
    /************************************/
    //              Publish             //
    /************************************/

    void PublishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr);
    bool PublishColorCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr);
    bool PublishCentroids(const ros::Publisher *in_publisher, const uav_msgs::Centroids &in_centroids);
    bool PublishCloudClusters(const ros::Publisher* in_publisher, const uav_msgs::CloudClusterArray& in_clusters);
    bool PublishDetectedObjects(const uav_msgs::CloudClusterArray &in_clusters);



    /************************************/
    //               etc                //     
    /************************************/
    void getParam();
    void GenerateColors(std::vector<Scalar>& colors, size_t count, size_t factor = 100);
    void DownsamplePoints(const Mat& src, Mat& dst, size_t count);

};

#endif