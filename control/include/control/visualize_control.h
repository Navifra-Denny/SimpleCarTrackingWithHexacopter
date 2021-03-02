#include "ros/ros.h"
#include <ros/spinner.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include "uav_msgs/PolyfitLane.h"
#include "uav_msgs/CarState.h"
#include "nav_msgs/Odometry.h"
#include "uav_msgs/Roi.h"
#include "uav_msgs/TargetWaypoints.h"
#include "uav_msgs/VehicleStateArray.h"
#include "uav_msgs/VehicleState.h"

#include "control/utils.h"
#include "control/generate_waypoints.h"

struct MetaMarkers{
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
    ros::Subscriber m_target_state_sub;
    ros::Subscriber m_ego_state_sub;
	ros::Subscriber m_target_waypoints_sub;

    // ros::Subscriber m_roi_waypoints_sub;
    // ros::Subscriber m_roi_position_sub;
    // ros::Subscriber m_roi_box_sub;
    
    // publisher
    ros::Publisher m_markers_pub;

    // timer
    ros::Timer m_visualize_control_timer;

    // Parm
    std::string m_ugv_name_param;
    std::string m_uav_name_param;

    control::Utils m_utils;
    MetaMarkers m_ego_markers;
    std::vector<MetaMarkers> m_targets_markers;
    MetaMarkers m_waypoints_markers;
    MetaMarkers m_roi_markers;

    // Marker Init
    void InitFlag();
    bool GetParam();
    void InitROS();
    void MarkerInit();

    // Callback
    void TimerCallback(const ros::TimerEvent& event);

    void EgoVehicleStateCallback(const uav_msgs::VehicleState::ConstPtr &ego_vehicle_ptr);
    void TargetVehicleStatesCallback(const uav_msgs::VehicleStateArray::ConstPtr &target_vehicle_ptr);
    void WaypointsCallback(const uav_msgs::TargetWaypoints::ConstPtr &target_wp_ptr);
    
    void ROIWaypointsCallback(const uav_msgs::PolyfitLane::ConstPtr &roi_waypoints);
    void ROICurrPositionCallback(const nav_msgs::Odometry::ConstPtr &current_point_ptr);
    void ROICallback(const uav_msgs::Roi::ConstPtr &current_point_ptr);
};