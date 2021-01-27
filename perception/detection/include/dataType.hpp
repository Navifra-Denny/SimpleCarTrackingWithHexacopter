#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include <cmath>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
 
#include "uav_msgs/Centroids.h"
#include "uav_msgs/CloudCluster.h"
#include "uav_msgs/CloudClusterArray.h"
#include "uav_msgs/DetectedObject.h"
#include "uav_msgs/DetectedObjectArray.h"
#include "uav_msgs/DTLane.h"
#include "uav_msgs/Lane.h"
#include "uav_msgs/LaneArray.h"
#include "uav_msgs/Waypoint.h"
#include "uav_msgs/WaypointState.h"

#include "jsk_rviz_plugins/Pictogram.h"

#define __APP_NAME__ "euclidean_clustering"
