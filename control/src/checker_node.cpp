#include "ros/ros.h"
#include <ros/spinner.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/HomePosition.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPose.h>
#include <sensor_msgs/NavSatFix.h>

#include "uav_msgs/TargetWP.h"
#include "uav_msgs/UavStatus.h"
#include "uav_msgs/Param.h"

class Checker{
public:
    Checker();
    ~Checker();

private:
    ros::NodeHandle m_nh;

    // Initialize subscriber
    ros::Subscriber m_px4_state_sub;
    ros::Subscriber m_waypoints_sub;
    ros::Subscriber m_current_local_pose_sub;
    ros::Subscriber m_current_global_pose_sub;
    ros::Subscriber m_home_position_sub;

    // Initialize publisher
    ros::Publisher m_uav_status_pub;

    // Initialize timer
    ros::Timer m_checker_timer;

    uav_msgs::UavStatus m_uav_status;
    
    // Param
    bool m_is_debug_mode_param;
    float m_setpoint_pub_interval_param;

    bool GetParam();
    void InitROS();

    void CheckerTimerCallback(const ros::TimerEvent& event);
    
    void StateCallback(const mavros_msgs::State::ConstPtr &state_ptr);
    void WaypointsCallback(const uav_msgs::TargetWP::ConstPtr &waypoints_ptr);
    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &local_position_ptr);
    void EgoVehicleGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &global_position_ptr);
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_position_ptr);
};

Checker::Checker() :
    m_setpoint_pub_interval_param(NAN)
{
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();
}

Checker::~Checker()
{}

bool Checker::GetParam()
{
    m_nh.getParam("checker_node/is_debug_mode", m_is_debug_mode_param);
    m_nh.getParam("checker_node/setpoint_pub_interval", m_setpoint_pub_interval_param);

    if (__isnan(m_setpoint_pub_interval_param)) { ROS_ERROR_STREAM("m_setpoint_pub_interval_param is NAN"); return false; }

    m_uav_status.param.is_debug_mode = m_is_debug_mode_param;
    m_uav_status.param.setpoint_pub_interval = m_setpoint_pub_interval_param;

    return true;
}

void Checker::InitROS()
{
    // Initialize subscriber
    m_px4_state_sub = m_nh.subscribe<mavros_msgs::State>("mavros/state", 10, boost::bind(&Checker::StateCallback, this, _1));
    m_waypoints_sub = m_nh.subscribe<uav_msgs::TargetWP>("/control/generate_waypoints_node/target_waypoints", 10, boost::bind(&Checker::WaypointsCallback, this, _1));
    m_current_local_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&Checker::EgoVehicleLocalPositionCallback, this, _1));
    m_current_global_pose_sub = m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&Checker::EgoVehicleGlobalPositionCallback, this, _1));
    // m_home_position_sub = m_nh.subscribe<geographic_msgs::GeoPoint>("/control/tf_broadcaster_node/home", 10, boost::bind(&Checker::HomePositionCallback, this, _1));
    m_home_position_sub = m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&Checker::HomePositionCallback, this, _1));

    // Initialize publisher
    m_uav_status_pub = m_nh.advertise<uav_msgs::UavStatus>("/control/checker_node/uav_status", 10);

    // Initialize timer
    m_checker_timer = m_nh.createTimer(ros::Duration(0.1), &Checker::CheckerTimerCallback, this);
}


void Checker::StateCallback(const mavros_msgs::State::ConstPtr &state_ptr)
{
    m_uav_status.mode = state_ptr->mode;
}

void Checker::WaypointsCallback(const uav_msgs::TargetWP::ConstPtr &waypoints_ptr)
{
    m_uav_status.state.global_to_local = waypoints_ptr->state.global_to_local;
    m_uav_status.state.is_global = waypoints_ptr->state.is_global;
    m_uav_status.state.is_detected = waypoints_ptr->state.is_detected;
    m_uav_status.state.is_hover = waypoints_ptr->state.is_hover;

    if (m_uav_status.state.is_global){
        if (waypoints_ptr->global.poses.size() != 0){
            m_uav_status.global_wp = waypoints_ptr->global.poses.back().pose;
        }
    }
    else{
        if (waypoints_ptr->local.poses.size() != 0){
            m_uav_status.local_wp = waypoints_ptr->local.poses.back();
        }
    }
}

void Checker::EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &local_position_ptr)
{
    m_uav_status.curr_local = local_position_ptr->pose;
}

void Checker::EgoVehicleGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &global_position_ptr)
{
    m_uav_status.curr_global.position.latitude = global_position_ptr->latitude;
    m_uav_status.curr_global.position.longitude = global_position_ptr->longitude;
    m_uav_status.curr_global.position.altitude = global_position_ptr->altitude;
}

void Checker::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_position_ptr)
{
    m_uav_status.param.init_gps_lat = home_position_ptr->geo.latitude;
    m_uav_status.param.init_gps_lon = home_position_ptr->geo.longitude;
    m_uav_status.param.init_gps_alt = home_position_ptr->geo.altitude;
}


void Checker::CheckerTimerCallback(const ros::TimerEvent& event)
{
    m_uav_status_pub.publish(m_uav_status);
}


int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "checker_node");
    Checker checker;

    ros::spin();
    return 0;
}