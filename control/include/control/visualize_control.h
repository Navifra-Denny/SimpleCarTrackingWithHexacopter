#include "ros/ros.h"
#include <ros/spinner.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <novatel_oem7_msgs/INSPVA.h>

#include "mavros_msgs/HomePosition.h"

#include "uav_msgs/PolyfitLane.h"
#include "uav_msgs/CarState.h"
#include "nav_msgs/Odometry.h"
#include "uav_msgs/Roi.h"
#include "uav_msgs/TargetWP.h"
#include "uav_msgs/Trajectory.h"
#include "control/utils.h"

struct Line{
    visualization_msgs::MarkerArray self;
    visualization_msgs::Marker line_strip;
    visualization_msgs::Marker orientation;
    visualization_msgs::Marker center_point;
    visualization_msgs::Marker roi;
    visualization_msgs::Marker txt;

    bool is_line_strip_add;
    bool is_orientation_add;
    bool is_center_point_add;
    bool is_roi_add;
    bool is_txt_add;
};

class VisualizeControl
{
public:
    VisualizeControl();
    virtual ~VisualizeControl();
private:
    // Node Handler
	ros::NodeHandle m_nh;

	// subscriber
    ros::Subscriber m_home_position_sub;
	ros::Subscriber m_target_trajectory_sub;
	ros::Subscriber m_ego_trajectory_sub;
    ros::Subscriber m_target_vehicle_global_position_sub;
    ros::Subscriber m_target_vehicle_local_ned_position_sub;
	ros::Subscriber m_target_waypoints_sub;
    ros::Subscriber m_roi_waypoints_sub;
    ros::Subscriber m_roi_position_sub;
    ros::Subscriber m_ego_vehicle_local_pose_sub;
    ros::Subscriber m_ego_vehicle_local_vel_sub;
    ros::Subscriber m_roi_box_sub;
    
    // publisher
    ros::Publisher m_markers_pub;
    ros::Timer m_roi_line_update_timer;

    // Parm
    std::string m_ugv_name_param;
    std::string m_uav_name_param;
    double m_target_height_m_param;

    // Flag
    bool m_is_home_set;

    control::Utils m_utils;
    double m_ego_vehicle_speed_kmh;
    Line m_target_trajectory;
    Line m_ego_trajectory;
    Line m_target_waypoints;
    Line m_roi_line;

    geographic_msgs::GeoPoint m_home_position;

    // Marker Init
    void InitFlag();
    bool GetParam();
    void InitROS();
    void MarkerInit();

    // Callback
    void TimerCallback(const ros::TimerEvent& event);

    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr);
    void EgoVehicleLocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &twist_ptr);
    void EgoTrajectoryCallback(const uav_msgs::Trajectory::ConstPtr &trajectory_ptr);

    void TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr);
    void TargetVehicleLocalNedPositionCallback(const uav_msgs::CarState::ConstPtr &current_point_ptr);
    void TargetTrajectoryCallback(const uav_msgs::Trajectory::ConstPtr &trajectory_ptr);
    
    void WaypointsCallback(const uav_msgs::TargetWP::ConstPtr &target_wp_ptr);
    
    void ROIWaypointsCallback(const uav_msgs::PolyfitLane::ConstPtr &roi_waypoints);
    void ROICurrPositionCallback(const nav_msgs::Odometry::ConstPtr &current_point_ptr);
    void ROICallback(const uav_msgs::Roi::ConstPtr &current_point_ptr);
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
};