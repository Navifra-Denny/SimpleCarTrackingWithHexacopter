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

#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>

#include "uav_msgs/CarState.h"
#include "control/utils.h"

using Vector3f = Eigen::Vector3f;

struct VehicleState{
    geometry_msgs::PoseStamped local;
    geographic_msgs::GeoPoseStamped global; // [deg/1e-7]

    geometry_msgs::PoseArray local_trajectory;
    geographic_msgs::GeoPath global_trajectory;

    int g_speed_raw;                                // [mm/s]
    double g_speed;                                 // [m/s]
    int heading_raw;                                // [deg/1e-5]
    double heading_deg;
    double heading_rad;
    double east;
    double north;
};

class GenerateWaypoints
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    control::Utils m_utils;
    VehicleState m_target_vehicle;
    VehicleState m_ego_vehicle;
	
public:
    GenerateWaypoints();
    virtual ~GenerateWaypoints();

    void GetInputWaypoints();
    void GetDesiredWaypoints();

private:
    // subscriber
	ros::Subscriber m_target_vehicle_local_state_sub;
	ros::Subscriber m_target_vehicle_global_position_sub;
    ros::Subscriber m_current_local_pose_sub;
    ros::Subscriber m_z_target_sub;

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

    // flag
    bool m_is_detected;
    bool m_is_global;
    bool m_is_hover;
    bool m_is_golbal_to_local;
    bool m_is_z_changed;


	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

    ros::Time m_last_detected_time;
    geometry_msgs::PoseArray m_target_wp_local;
    geographic_msgs::GeoPath m_target_wp_global;
    float m_z_offset_m;

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
    void ZOffsetCallback(const geometry_msgs::Point::ConstPtr &point_ptr);

    bool AddPointToTrajectory(geometry_msgs::PoseArray &pose_array, geometry_msgs::PoseStamped &curr_pose_stamped);
    bool AddTargetWaypoint(geometry_msgs::PoseArray &pose_array, geometry_msgs::PoseStamped &curr_pose_stamped);
    geometry_msgs::Pose GenTargetWaypoint(geometry_msgs::Pose &curr_pose);

    void Publish(geometry_msgs::PoseArray &target_trajectory, geometry_msgs::PoseArray &ego_trajectory, geometry_msgs::PoseArray &target_poses);
    
    bool IsValid(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point curr_position);
    bool IsValid(std::vector<geometry_msgs::Pose> &poses);
};

#endif // __GENERATE_WAYPOINTS_H__