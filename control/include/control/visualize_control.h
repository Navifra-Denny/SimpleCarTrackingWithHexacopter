#include "ros/ros.h"
#include <ros/spinner.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <novatel_oem7_msgs/INSPVA.h>

#include "uav_msgs/PolyfitLane.h"
#include "uav_msgs/CarState.h"
#include "nav_msgs/Odometry.h"
#include "uav_msgs/Roi.h"
#include "uav_msgs/TargetWP.h"
#include "uav_msgs/Trajectory.h"

struct Line{
    visualization_msgs::MarkerArray self;
    visualization_msgs::Marker line_strip;
    visualization_msgs::Marker orientation;
    visualization_msgs::Marker center_point;
    visualization_msgs::Marker roi;
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
	ros::Subscriber m_target_trajectory_sub;
	ros::Subscriber m_ego_trajectory_sub;
	ros::Subscriber m_target_vehicle_local_ned_position_sub;
	ros::Subscriber m_target_waypoints_sub;
    ros::Subscriber m_roi_waypoints_sub;
    ros::Subscriber m_roi_position_sub;
    ros::Subscriber m_ego_vehicle_local_pose_sub;
    ros::Subscriber m_roi_box_sub;
    
    // publisher
    ros::Publisher m_markers_pub;
    ros::Timer m_roi_line_update_timer;

    // Parm
    std::string m_ugv_name_param;
    std::string m_uav_name_param;

    // Marker Init
    bool GetParam();
    void InitROS();
    void MarkerInit();

    // Callback
    void TargetTrajectoryCallback(const uav_msgs::Trajectory::ConstPtr &trajectory_ptr);
    void EgoTrajectoryCallback(const uav_msgs::Trajectory::ConstPtr &trajectory_ptr);
    void WaypointsCallback(const uav_msgs::TargetWP::ConstPtr &target_wp_ptr);

    void TargetVehicleLocalNedPositionCallback(const uav_msgs::CarState::ConstPtr &current_point_ptr);
    void ROIWaypointsCallback(const uav_msgs::PolyfitLane::ConstPtr &roi_waypoints);
    void ROICurrPositionCallback(const nav_msgs::Odometry::ConstPtr &current_point_ptr);
    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr);
    void EgoVehicleGlobalPositionCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &current_pose_ptr);
    void ROICallback(const uav_msgs::Roi::ConstPtr &current_point_ptr);
    void TimerCallback(const ros::TimerEvent& event);

    Line m_target_trajectory;
    Line m_ego_trajectory;
    Line m_target_waypoints;
    Line m_roi_line;
};