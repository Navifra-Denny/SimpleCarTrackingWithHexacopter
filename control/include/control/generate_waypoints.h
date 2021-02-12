#ifndef __GENERATE_WAYPOINTS_H__
#define __GENERATE_WAYPOINTS_H__

#include "ros/ros.h"
#include <ros/spinner.h>
#include "Eigen/Dense"
#include <deque>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <novatel_oem7_msgs/INSPVA.h>

#include <geographic_msgs/GeoPointStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include "uav_msgs/CarState.h"
#include "control/utils.h"

#define SEMI_MAJOR_AXIS 6378137.0       // semi-major axis [m]
#define SEMI_MINOR_AXIS 6356752.314245  // semi-minor axis [m]
using Vector3f = Eigen::Vector3f;

class GenerateWaypoints
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    control::Utils m_utils;
    control::VehicleState m_target_vehicle;
	
public:
    GenerateWaypoints();
    virtual ~GenerateWaypoints();

    void GetInputWaypoints();
    void GetDesiredWaypoints();

private:
    // subscriber
	ros::Subscriber m_target_vehicle_local_state_sub;
	ros::Subscriber m_target_vehicle_global_position_sub;

	// publisher
	ros::Publisher m_input_local_waypoints_pub;
    ros::Publisher m_desired_local_waypoints_pub;
    ros::Publisher m_desired_global_waypoints_pub;
    ros::Publisher m_global_to_enu_target_vehicle_pose_pub;
    ros::Publisher m_world_enu_desired_waypoints_pub;

    // service client
    ros::ServiceClient m_desired_waypoints_srv_client;

    // param
    std::string m_vehicle_name_param;
    float m_x_offset_m_param;
    float m_z_offset_m_param;
    float m_distance_thresh_param;
    float m_init_gps_lat_param;
    float m_init_gps_lon_param;
    float m_init_gps_alt_param;

    // const value
    double m_K_LON;
    double m_K_LAT;

private: // function
    void GetParam();
    void RosInit();
    void InitTargetVehicle();

    void TargetVehicleLocalStateCallback(const uav_msgs::CarState::ConstPtr &car_state_ptr);
    bool ConvertStateToLocalWaypoints(geometry_msgs::PoseStamped pose_stamped);
    void TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &current_pose_ptr);
    bool ConvertStateToGlobalWaypoints(geometry_msgs::PoseStamped pose_stamped);

    geometry_msgs::PoseStamped CreateDesiredLocalWaypoint(geometry_msgs::PoseStamped pose_stamped);
    geometry_msgs::PoseArray ConvertWorldEnu(geometry_msgs::PoseArray source_pose_array);

private: // attribute
    std::vector<geometry_msgs::PoseStamped> m_input_local_waypoints;
    std::vector<geometry_msgs::PoseStamped> m_desired_local_waypoints;
};

#endif // __GENERATE_WAYPOINTS_H__