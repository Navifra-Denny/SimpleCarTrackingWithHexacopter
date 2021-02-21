#ifndef __GENERATE_WAYPOINTS_H__
#define __GENERATE_WAYPOINTS_H__

#include "ros/ros.h"
#include <ros/spinner.h>
#include "Eigen/Dense"
#include <deque>
#include <novatel_oem7_msgs/INSPVA.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>

#include <std_msgs/String.h>
#include <mavros_msgs/HomePosition.h>
#include "uav_msgs/TargetWP.h"
#include "uav_msgs/CarState.h"
#include "uav_msgs/Offset.h"
#include "control/utils.h"

using Vector3f = Eigen::Vector3f;

namespace control
{
struct VehicleState{
    geometry_msgs::PoseStamped local;
    geographic_msgs::GeoPoseStamped global; // [deg/1e-7]

    geometry_msgs::PoseArray local_trajectory;
    geographic_msgs::GeoPath global_trajectory;

    geometry_msgs::Twist velocity;           // [m/s]
    int g_speed_raw;                                // [mm/s]
    int heading_raw;                                // [deg/1e-5]
    double heading_deg;
    double heading_rad;
    double east;
    double north;
};
}

class GenerateWaypoints
{	
public:
    GenerateWaypoints();
    virtual ~GenerateWaypoints();

private:
    // Node Handler
	ros::NodeHandle m_nh;

    // subscriber
	ros::Subscriber m_target_vehicle_local_state_sub;
	ros::Subscriber m_target_vehicle_global_position_sub;
    ros::Subscriber m_current_local_pose_sub;
    ros::Subscriber m_offset_sub;
    ros::Subscriber m_chatter_sub;
    ros::Subscriber m_home_position_sub;

	// publisher
	ros::Publisher m_target_trajectory_pub;
	ros::Publisher m_ego_trajectory_pub;
	ros::Publisher m_target_waypoints_pub;

    // Timer
    ros::Timer m_generate_waypoints_timer;

    // param
    float m_x_offset_m_param;
    float m_z_offset_m_param;
    std::string m_vehicle_name_param;
    float m_distance_thresh_param;
    float m_target_wp_pub_interval_param;
    float m_detected_dead_band_param;
    bool m_global_to_local_param;
    float m_alt_offset_m_param;
    float m_target_height_m_param;

    // flag
    bool m_is_detected;
    bool m_is_global;
    bool m_is_hover;
    bool m_is_offset_changed;
    bool m_is_home_set;

    control::Utils m_utils;
    control::VehicleState m_target_vehicle;
    control::VehicleState m_ego_vehicle;
	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

    geographic_msgs::GeoPoint m_home_position;
    ros::Time m_last_detected_time;
    // geometry_msgs::PoseArray m_target_wp_local;
    // geographic_msgs::GeoPath m_target_wp_global;
    uav_msgs::TargetWP m_target_wp;
    // uav_msgs::TargetWP m_target_wp_global;
    float m_z_offset_m;
    float m_x_offset_m;
    float m_alt_offset_m;

private: // function
    bool GetParam();
    bool InitROS();
    bool InitFlag();
    bool InitTargetVehicle();
    
    void GenerateWaypointsTimerCallback(const ros::TimerEvent& event);

    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr);
    void TargetVehicleLocalStateCallback(const uav_msgs::CarState::ConstPtr &car_state_ptr);
    // void TargetVehicleLocalStateCallback(); // Tracking lidar callback
    void TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &current_pose_ptr);
    void OffsetCallback(const uav_msgs::Offset::ConstPtr &point_ptr);
    void ChatterCallback(const std_msgs::String::ConstPtr &string_ptr);
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);

    bool AddPointToTrajectory(geometry_msgs::PoseArray &pose_array, geometry_msgs::PoseStamped &curr_pose_stamped);
    bool AddTargetWaypoint(uav_msgs::TargetWP &target_wp, geometry_msgs::PoseStamped &curr_pose_stamped);
    bool AddTargetWaypoint(uav_msgs::TargetWP &target_wp, geometry_msgs::PoseStamped &curr_pose_stamped, geometry_msgs::Twist &target_vel);
    bool AddTargetWaypoint(uav_msgs::TargetWP &target_wp, geographic_msgs::GeoPoseStamped &curr_geo_pose_stamped, geometry_msgs::Twist &target_vel);
    geometry_msgs::Pose GenTargetWaypoint(geometry_msgs::Pose &curr_pose);
    geographic_msgs::GeoPoseStamped GenTargetWaypoint(geographic_msgs::GeoPoseStamped &curr_geo_pose_stamped);

    void Publish();
    bool IsValid(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point curr_position);
    bool IsValid(std::vector<geographic_msgs::GeoPoseStamped> &poses, geographic_msgs::GeoPoint curr_position);
    bool IsValid(std::vector<geometry_msgs::Pose> &poses);
    bool IsValid(std::vector<geographic_msgs::GeoPoseStamped> &poses);
};

#endif // __GENERATE_WAYPOINTS_H__