#ifndef __GENERATE_WAYPOINTS_H__
#define __GENERATE_WAYPOINTS_H__

#include "ros/ros.h"
#include <ros/spinner.h>
#include "Eigen/Dense"
#include <deque>
#include <vector>
#include <novatel_oem7_msgs/INSPVA.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>

#include "uav_msgs/TargetWaypoints.h"
#include "uav_msgs/CarState.h"
#include "uav_msgs/Chat.h"
#include "uav_msgs/TargetState.h"
#include "uav_msgs/VehicleState.h"
#include "uav_msgs/GenerateWaypointState.h"
#include "control/utils.h"

using Vector3f = Eigen::Vector3f;

namespace control
{
enum class DetectionTool : int
{
    AirSim,
    LiDAR,
    GPS,

    ItemNum
};

struct VehicleState{
    geometry_msgs::PoseStamped local;
    geographic_msgs::GeoPoseStamped global; // [deg/1e-7]
    geometry_msgs::Twist velocity;           // [m/s]

    geometry_msgs::PoseArray local_trajectory;
    geographic_msgs::GeoPath global_trajectory;
};

struct TargetState : VehicleState{
    bool is_detected;
    ros::Time last_detected_time;
    DetectionTool tool;
};
}

using namespace control;

class GenerateWaypoints
{	
public:
    GenerateWaypoints();
    virtual ~GenerateWaypoints();

private:
    // Node Handler
	ros::NodeHandle m_nh;

    // subscriber
	ros::Subscriber m_airsim_based_target_local_state_sub;
	ros::Subscriber m_lidar_based_target_local_state_sub;
	ros::Subscriber m_gps_based_target_global_position_sub;

    ros::Subscriber m_ego_local_pose_sub;
    ros::Subscriber m_ego_global_pose_sub;
    ros::Subscriber m_ego_local_vel_sub;

    ros::Subscriber m_chatter_sub;
    ros::Subscriber m_init_pose_sub;
    ros::Subscriber m_home_position_sub;

	// publisher
	ros::Publisher m_target_states_pub;
	ros::Publisher m_ego_state_pub;
	ros::Publisher m_target_waypoints_pub;
	ros::Publisher m_gen_wp_state_pub;

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
    bool m_detection_tool_lidar_param;
    bool m_detection_tool_gps_param;
    bool m_detection_tool_airsim_param;

    // flag
    bool m_is_hover;
    bool m_is_reached;
    bool m_is_global;
    bool m_global_to_local;
    bool m_is_offset_changed;
    bool m_is_heading_changed;    
    bool m_is_home_set;

    int SELECTED_TOOL;

    Utils m_utils;
    std::vector<TargetState> m_target_vehicles;
    VehicleState m_ego_vehicle;
    
	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

    geometry_msgs::Quaternion m_target_orientation;
    geographic_msgs::GeoPoint m_home_position;
    uav_msgs::TargetWaypoints m_target_wp;

    double m_complement_x;
    double m_complement_y;

private: // function
    bool GetParam();
    bool InitROS();
    bool InitFlag();
    bool InitTargetVehicle();
    
    void GenerateWaypointsTimerCallback(const ros::TimerEvent& event);

    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr);
    void EgoVehicleGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &current_pose_ptr);
    void EgoVehicleLocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &twist_ptr);
    void TargetVehicleLocalStateCallback(const uav_msgs::CarState::ConstPtr &target_state_ptr);
    void LTargetVehicleLocalStateCallback(const uav_msgs::TargetState::ConstPtr &target_state_ptr);
    void TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &current_pose_ptr);
    void ChatterCallback(const uav_msgs::Chat::ConstPtr &string_ptr);
    void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_ptr);
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);

    bool AddPoseToTrajectory(geometry_msgs::PoseArray &pose_array, geometry_msgs::PoseStamped &curr_pose_stamped);
    bool AddTargetWaypoint(uav_msgs::TargetWaypoints &target_wp, geometry_msgs::PoseStamped &curr_pose_stamped);
    bool AddTargetWaypoint(uav_msgs::TargetWaypoints &target_wp, geometry_msgs::PoseStamped &curr_pose_stamped, geometry_msgs::Twist &target_vel);
    bool AddTargetWaypoint(uav_msgs::TargetWaypoints &target_wp, geographic_msgs::GeoPoseStamped &curr_geo_pose_stamped, geometry_msgs::Twist &target_vel);
    geometry_msgs::Pose GenTargetWaypoint(geometry_msgs::Pose &curr_pose, geometry_msgs::Twist &target_vel);
    geographic_msgs::GeoPoseStamped GenTargetWaypoint(geographic_msgs::GeoPoseStamped &curr_geo_pose_stamped);

    void Publish();
    bool IsValid(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point curr_position, bool is_waypoint=false);
    bool IsValid(std::vector<geographic_msgs::GeoPoseStamped> &poses, geographic_msgs::GeoPoint curr_position);
    bool IsValid(std::vector<geometry_msgs::Pose> &poses);
    bool IsValid(std::vector<geographic_msgs::GeoPoseStamped> &poses);
    bool IsReached(uav_msgs::TargetWaypoints &wp, geometry_msgs::Point curr_position);
    bool IsReached(uav_msgs::TargetWaypoints &wp, geographic_msgs::GeoPoint curr_position);
    bool IsDetected(TargetState& target_state, bool is_selected);
    bool IsGlobalToLocal();
};

#endif // __GENERATE_WAYPOINTS_H__