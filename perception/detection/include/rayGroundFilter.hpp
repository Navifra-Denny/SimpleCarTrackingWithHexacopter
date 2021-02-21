#ifndef __RAYGROUNDFILTER_HPP__
#define __RAYGROUNDFILTER_HPP__

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define USE_ATAN_APPROXIMATION

/// pi
constexpr float PI = 3.14159265359F;
/// pi/2
constexpr float PI_2 = 1.5707963267948966F;

class RayGroundFilter
{
private:

    // Get Param 
    double _general_max_slope;            // degrees
    double _local_max_slope;              // degrees
    double _radial_divider_angle;         // distance in rads between dividers
    double _concentric_divider_distance;  // distance in meters between concentric divisions
    double _min_height_threshold;         // minimum height threshold regardless the slope, useful for close points
    double _clipping_height;              // the points higher than this will be removed from the input cloud.
    double _min_point_distance;           // minimum distance from the origin to consider a point as valid
    double _reclass_distance_threshold;   // distance between points at which re classification will occur
    double _sensor_height;
    std::string _lidar_frame_id;

    // Param
    size_t _radial_dividers_num;
    size_t _concentric_dividers_num;

    struct PointRH
    {
        float height;
        float radius;  // cylindric coords on XY Plane
        void* original_data_pointer;

        PointRH(float height, float radius, void* original_data_pointer)
        : height(height), radius(radius), original_data_pointer(original_data_pointer)
        {}
    };
    
    typedef std::vector<PointRH> PointCloudRH;   

public:
    // Constructor
    RayGroundFilter();
    virtual ~RayGroundFilter();
    
    // Get param init
    void initParam(
        double general_max_slope,
        double local_max_slope,
        double radial_divider_angle,
        double concentric_divider_distance,
        double min_height_threshold,
        double clipping_height,
        double min_point_distance,
        double reclass_distance_threshold,
        std::string lidar_frame_id,
        double sensor_height
    );


    bool removeFloor_rayGroundFilter(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_cloud, const sensor_msgs::PointCloud2::Ptr& nofloor_cloud_ptr,
                                                  const sensor_msgs::PointCloud2::Ptr& floor_cloud_ptr);

private: 

    // Processing main function
    // Convert ConstPtr to Ptr type 
    bool TransformPointCloud(const sensor_msgs::PointCloud2::ConstPtr& in_cloud_ptr, const sensor_msgs::PointCloud2::Ptr& out_cloud_ptr);


    bool ConvertAndTrim(const sensor_msgs::PointCloud2::Ptr in_transformed_cloud,
                      const double in_clip_height,
                      double in_min_distance,
                      std::vector<PointCloudRH>* out_radial_ordered_clouds,
                      std::vector<void*>* out_no_ground_ptrs);


    bool ClassifyPointCloud(const std::vector<PointCloudRH>& in_radial_ordered_clouds,
                                            const size_t in_point_count,
                                            std::vector<void*>* out_ground_ptrs,
                                            std::vector<void*>* out_no_ground_ptrs);


    bool FilterROSMsg(const sensor_msgs::PointCloud2ConstPtr in_sensor_cloud,
                      const std::vector<void*>& in_selector,
                      const sensor_msgs::PointCloud2::Ptr out_filtered_msg);


    // Function for Intermediate Calculation
    bool is_big_endian(void);
    float ReverseFloat(float inFloat);
    float fast_atan2(float y, float x);


};
#endif