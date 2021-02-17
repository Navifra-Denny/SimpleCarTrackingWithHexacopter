#ifndef __MAVROS_OFFB_H__
#define __MAVROS_OFFB_H__

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <novatel_oem7_msgs/INSPVA.h>
#include <std_msgs/String.h>

#include "uav_msgs/CarState.h"
#include "uav_msgs/UavStatus.h"
#include "uav_msgs/TargetWP.h"

#include "control/generate_waypoints.h"
#include "control/utils.h"

namespace mavros{

class Offboard
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    control::Utils m_utils;
    VehicleState m_ego_vehicle;

public:
    Offboard();
    virtual ~Offboard();

private:
    // Subscriber
    ros::Subscriber m_state_sub;
    ros::Subscriber m_target_waypoints_sub;

    ros::Subscriber m_desired_local_waypoints_sub;
    ros::Subscriber m_desired_global_waypoint_sub;

    ros::Subscriber m_current_global_pose_sub;
    ros::Subscriber m_debugging_sub;

    // Publisher
    ros::Publisher m_local_pose_pub;
    ros::Publisher m_global_pose_pub;
    ros::Publisher m_gp_origin_pub;
    ros::Publisher m_home_pub;
    
    // ServiceClient
    ros::ServiceClient m_arming_serv_client;
    ros::ServiceClient m_set_mode_serv_client;

    ros::Timer m_timer;

    // param
    float m_setpoint_pub_interval_param;
    float m_init_gps_lat_param;
    float m_init_gps_lon_param;
    float m_init_gps_alt_param;
    float m_z_offset_param_m;
    bool m_use_global_setpoint_param;
    bool m_is_debug_mode_param;

    // flag
    bool m_get_gps_origin;
    bool m_is_hover;
    bool m_is_detected;
    bool m_is_global;

    ros::Time m_last_request_time;
    ros::Time m_last_detected_object_time;

    mavros_msgs::HomePosition m_home_pos;
    geographic_msgs::GeoPointStamped m_gp_origin;
    mavros_msgs::State m_current_status;
    geometry_msgs::PoseStamped m_local_setpoint;
    geographic_msgs::GeoPoseStamped m_global_setpoint;
    std::string m_debugging_msg;

	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;
    uav_msgs::UavStatus m_uav_status_msg;

private: // function
    bool GetParam();
    bool InitFlag();
    bool InitRos();
    bool InitClient();
    
    void OffboardTimeCallback(const ros::TimerEvent& event);
    
    void StatusCallback(const mavros_msgs::State::ConstPtr &state_ptr);
    void TargetWaypointsCallback(const uav_msgs::TargetWP::ConstPtr &target_wp_ptr);
    void DesiredGlobalWaypointCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_ptr);
    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr);
    void EgoVehicleGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &current_pose_ptr);
    void DebuggingStringCallback(const std_msgs::String::ConstPtr &debugging_msgs);

    void OffboardReConnection();
    void PublishSetpoint();
};
}
#endif //  __MAVROS_OFFB_H__