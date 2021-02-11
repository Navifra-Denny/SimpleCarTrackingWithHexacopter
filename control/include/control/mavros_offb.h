#ifndef __MAVROS_OFFB_H__
#define __MAVROS_OFFB_H__

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

#include <nav_msgs/Odometry.h>

#include "uav_msgs/CarState.h"
#include "control/utils.h"

namespace mavros{

class Offboard
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    control::Utils m_utils;
public:
    Offboard();
    virtual ~Offboard();

private:
    // Subscriber
    ros::Subscriber m_state_sub;
    ros::Subscriber m_desired_waypoints_sub;
    ros::Subscriber m_car_state_sub;
    //ros::Subscriber m_odom_sub;

    // Publisher
    ros::Publisher m_local_pose_pub;
    
    // ServiceClient
    ros::ServiceClient m_arming_serv_client;
    ros::ServiceClient m_set_mode_serv_client;

    //ros::Timer m_target_waypoint_pub;
    ros::Timer m_timer;

    // param
    float m_speed_ms_param;
    std::string m_ue_target_name_param;
    std::string m_uav_name_param;
    std::string m_target_vehicle_name_param;
    float m_setpoint_pub_interval_param;
    float m_dt_param;
    float m_init_pos_x_param;
    float m_init_pos_y_param;
    float m_init_pos_z_param;
    // float m_x_offset_m_param;
    // float m_z_offset_m_param;

    ros::Time m_last_request;

    mavros_msgs::State m_current_status;
    geometry_msgs::PoseStamped m_current_pose;
    geometry_msgs::PoseStamped m_target_pose;


private: // function
    void GetParam();
    void InitRos();
    void InitClient();
    
    void StatusCallback(const mavros_msgs::State::ConstPtr &state_ptr);
    void DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr pose_array);
    void CarStateCallback(const uav_msgs::CarState::ConstPtr &car_state_ptr);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_ptr);
    void TimerCallback(const ros::TimerEvent& event);

    void OffboardReConnection();
};
}
#endif //  __MAVROS_OFFB_H__