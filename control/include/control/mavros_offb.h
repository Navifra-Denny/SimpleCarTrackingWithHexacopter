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

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <novatel_oem7_msgs/INSPVA.h>

#include "uav_msgs/CarState.h"
#include "control/utils.h"

namespace mavros{
class Offboard
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    control::Utils m_utils;
    control::VehicleState m_ego_vehicle;

public:
    Offboard();
    virtual ~Offboard();

private:
    // Subscriber
    ros::Subscriber m_state_sub;
    ros::Subscriber m_desired_local_waypoints_sub;
    ros::Subscriber m_desired_global_waypoint_sub;
    ros::Subscriber m_current_local_pose_sub;
    ros::Subscriber m_current_global_pose_sub;

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
    float m_speed_ms_param;
    std::string m_uav_name_param;
    std::string m_target_vehicle_name_param;
    float m_setpoint_pub_interval_param;
    float m_dt_param;
    float m_init_pos_x_param;
    float m_init_pos_y_param;
    float m_init_pos_z_param;
    float m_init_gps_lat_param;
    float m_init_gps_lon_param;
    float m_init_gps_alt_param;
    bool m_use_global_setpoint_param;

    // flag
    bool m_is_ready_to_flight;
    bool m_get_gps_origin;

    ros::Time m_last_request;

    mavros_msgs::HomePosition m_home_pos;
    geographic_msgs::GeoPointStamped m_gp_origin;
    mavros_msgs::State m_current_status;
    geometry_msgs::PoseStamped m_local_setpoint;
    geographic_msgs::GeoPoseStamped m_global_setpoint;

	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

private: // function
    bool GetParam();
    bool InitFlag();
    bool InitRos();
    bool InitClient();
    
    void StatusCallback(const mavros_msgs::State::ConstPtr &state_ptr);
    void DesiredLocalWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array_ptr);
    void DesiredGlobalWaypointCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_ptr);
    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr);
    void EgoVehicleGlobalPositionCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &current_pose_ptr);
    void TimerCallback(const ros::TimerEvent& event);

    void OffboardReConnection();
};
}
#endif //  __MAVROS_OFFB_H__