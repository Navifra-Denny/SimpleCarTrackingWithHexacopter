#ifndef __CLUSTER_HPP__
#define __CLUSTER_HPP__

#include "dataType.hpp"

class Cluster
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_;
  pcl::PointXYZ min_point_;
  pcl::PointXYZ max_point_;
  pcl::PointXYZ average_point_;
  pcl::PointXYZ centroid_;
  double orientation_angle_;
  float length_, width_, height_;

  jsk_recognition_msgs::BoundingBox bounding_box_;
  geometry_msgs::PolygonStamped polygon_;

  std::string label_;
  int id_;
  int r_, g_, b_;

  Eigen::Matrix3f eigen_vectors_;
  Eigen::Vector3f eigen_values_;

  bool valid_cluster_;

public:
  /* \brief Constructor. Creates a Cluster object using the specified points in a PointCloud
   * \param[in] in_origin_cloud_ptr   Origin PointCloud
   * \param[in] in_cluster_indices   Indices of the Origin Pointcloud to create the Cluster
   * \param[in] in_id         ID of the cluster
   * \param[in] in_r           Amount of Red [0-255]
   * \param[in] in_g           Amount of Green [0-255]
   * \param[in] in_b           Amount of Blue [0-255]
   * \param[in] in_label         Label to identify this cluster (optional)
   * \param[in] in_estimate_pose    Flag to enable Pose Estimation of the Bounding Box
   * */
  void SetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr,
                const std::vector<int>& in_cluster_indices, std_msgs::Header in_ros_header, int in_id, int in_r,
                int in_g, int in_b, std::string in_label, bool in_estimate_pose);

  /* \brief Returns the autoware_msgs::CloudCluster message associated to this Cluster */
  void ToROSMessage(std_msgs::Header in_ros_header, uav_msgs::CloudCluster& out_cluster_message);

  Cluster();
  virtual ~Cluster();

  /* \brief Returns the pointer to the PointCloud containing the points in this Cluster */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud();
  /* \brief Returns the minimum point in the cluster */
  pcl::PointXYZ GetMinPoint();
  /* \brief Returns the maximum point in the cluster*/
  pcl::PointXYZ GetMaxPoint();
  /* \brief Returns the average point in the cluster*/
  pcl::PointXYZ GetAveragePoint();
  /* \brief Returns the centroid point in the cluster */
  pcl::PointXYZ GetCentroid();
  /* \brief Returns the calculated BoundingBox of the object */
  jsk_recognition_msgs::BoundingBox GetBoundingBox();
  /* \brief Returns the calculated PolygonArray of the object */
  geometry_msgs::PolygonStamped GetPolygon();
  /* \brief Returns the angle in radians of the BoundingBox. 0 if pose estimation was not enabled. */
  double GetOrientationAngle();
  /* \brief Returns the Id of the Cluster */
  int GetId();
  /* \brief Returns the Eigen Vectors of the cluster */
  Eigen::Matrix3f GetEigenVectors();
  /* \brief Returns the Eigen Values of the Cluster */
  Eigen::Vector3f GetEigenValues();
  /* \brief Returns if the Cluster is marked as valid or not*/
  bool IsValid();
  /* \brief Sets whether the Cluster is valid or not*/
  void SetValidity(bool in_valid);
 
};

typedef boost::shared_ptr<Cluster> ClusterPtr;

#endif