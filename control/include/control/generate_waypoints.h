#ifndef __GENERATE_WAYPOINTS_H__
#define __GENERATE_WAYPOINTS_H__

#include "ros/ros.h"
#include <ros/spinner.h>
#include "Eigen/Dense"
#include <deque>
#include <geometry_msgs/PoseArray.h>
#include "uav_msgs/CarState.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "control/utils.h"

using Vector3f = Eigen::Vector3f;


class GenerateWaypoints
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    control::Utils m_utils;

	
public:
    GenerateWaypoints();
    virtual ~GenerateWaypoints();

    void GetInputWaypoints();
    void GetDesiredWaypoints();

private:
    // subscriber
	ros::Subscriber m_car_state_sub;

	// publisher
	ros::Publisher m_input_waypoints_pub;
    ros::Publisher m_desired_waypoints_pub;
    ros::Publisher m_world_enu_input_waypoints_pub;
    ros::Publisher m_world_enu_desired_waypoints_pub;

    // service client
    ros::ServiceClient m_desired_waypoints_srv_client;

    // param
    std::string m_vehicle_name_param;
    float m_x_offset_m_param;
    float m_z_offset_m_param;

private: // function
    void GetParam();

    void CarStateCallback(const uav_msgs::CarState::ConstPtr &car_state);
    bool ConvertStateToWaypoints(geometry_msgs::PoseStamped pose_stamped);

    geometry_msgs::PoseStamped CreateDesiredWaypoint(geometry_msgs::PoseStamped pose_stamped);
    geometry_msgs::PoseArray ConvertWorldEnu(geometry_msgs::PoseArray source_pose_array);

private: // attribute
    std::vector<geometry_msgs::PoseStamped> m_input_waypoints;
    std::vector<geometry_msgs::PoseStamped> m_desired_waypoints;
};

#endif // __GENERATE_WAYPOINTS_H__