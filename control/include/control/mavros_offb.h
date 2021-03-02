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
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <novatel_oem7_msgs/INSPVA.h>
#include <std_msgs/String.h>

#include "uav_msgs/CarState.h"
#include "uav_msgs/TargetWaypoints.h"
#include "uav_msgs/Chat.h"
#include "uav_msgs/OffboardState.h"

#include "control/generate_waypoints.h"
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
    ros::Subscriber m_target_waypoints_sub;

    ros::Subscriber m_current_global_pose_sub;
    ros::Subscriber m_chatter_sub;

    // Publisher
    ros::Publisher m_local_pose_pub;
    ros::Publisher m_global_pose_pub;
    ros::Publisher m_velocity_pub;
    ros::Publisher m_offboard_state_pub;
    
    // ServiceClient
    ros::ServiceClient m_arming_serv_client;
    ros::ServiceClient m_set_mode_serv_client;

    ros::Timer m_timer;

    // param
    float m_setpoint_pub_interval_param;
    bool m_is_debug_mode_param;

    // flag
    bool m_is_global;
    bool m_is_debugging;
    
    ros::Time m_last_request_time;

    mavros_msgs::State m_current_status;
    geometry_msgs::PoseStamped m_local_setpoint;
    geographic_msgs::GeoPoseStamped m_global_setpoint;
    geometry_msgs::Twist m_vel_setpoint;

    std::string m_debugging_msg;
	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

private: // function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitClient();
    
    void OffboardTimeCallback(const ros::TimerEvent& event);
    
    void StatusCallback(const mavros_msgs::State::ConstPtr &state_ptr);
    void TargetWaypointsCallback(const uav_msgs::TargetWaypoints::ConstPtr &target_wp_ptr);
    void ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr);

    void OffboardReConnection();
    void Publish();
};
}
#endif //  __MAVROS_OFFB_H__