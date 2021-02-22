#include "control/extract_lane.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

ExtractLane::ExtractLane()
{
	// Set Param
	GetParam();

    // package, node, topic name
    std::string self_pkg_name = "/control";
    std::string self_node_name = "/extract_lane_node";
	std::string uav_state_sub_topic_name = "/airsim_node/" + m_vehicle_name_param + "/odom_local_ned";

	// Initialize subscriber 
	m_desired_local_waypoints_pub = m_nh.subscribe<geometry_msgs::PoseArray>(self_pkg_name + "/generate_waypoints_node/target_waypoints", 10, boost::bind(&ExtractLane::DesiredLocalWaypointsCallback, this, _1));
	m_uav_state_sub = m_nh.subscribe<nav_msgs::Odometry>(uav_state_sub_topic_name, 10, boost::bind(&ExtractLane::UavStateCallback, this, _1));

	// Initialize publisher 
	m_roi_box_pub = m_nh.advertise<uav_msgs::Roi> (self_pkg_name + self_node_name + "/roi", 1);
	m_roi_lane_pub = m_nh.advertise<uav_msgs::PolyfitLane> (self_pkg_name + self_node_name + "/poly_fit_lane", 1);
	m_evaulation_pub = m_nh.advertise<geometry_msgs::Point> (self_pkg_name + self_node_name + "/evaluation", 1);
	
	// m_poly_fit_lane_pub = m_nh.advertise<uav_msgs::PolyfitLaneData> ("polyfit_lanes", 10);
}

ExtractLane::~ExtractLane()
{ }

void ExtractLane::GetParam()
{
	m_nh.getParam("extract_lane_node/ROI_front_m", m_roi_front_param);
	m_nh.getParam("extract_lane_node/ROI_rear_m", m_roi_rear_param);
	m_nh.getParam("extract_lane_node/ROI_lateral_m", m_roi_lateral_param);
	m_nh.getParam("extract_lane_node/ROI_vertical_m", m_roi_vertical_param);
	m_nh.getParam("extract_lane_node/uav_name", m_vehicle_name_param);
}

void ExtractLane::UavStateCallback(const nav_msgs::Odometry::ConstPtr odm_ptr)
{
	m_curr_uav_position.header = odm_ptr->header;
	m_curr_uav_position.point.x = odm_ptr->pose.pose.position.x;
	m_curr_uav_position.point.y = odm_ptr->pose.pose.position.y;
	m_curr_uav_position.point.z = odm_ptr->pose.pose.position.z;

	auto longer = (m_roi_front_param > m_roi_rear_param) ? m_roi_front_param : m_roi_rear_param; 
	auto center = (m_roi_front_param + m_roi_rear_param)/2;
	auto complement_x = longer - center;
	
	m_roi_msg.header = odm_ptr->header;

	auto euler = m_utils.Quat2Euler(odm_ptr->pose.pose.orientation);
	auto roll = euler.r;
	auto pitch = euler.p;
	auto yaw = euler.y;
	m_roi_msg.pose.position.x = odm_ptr->pose.pose.position.x + complement_x*cos(pitch)*cos(yaw);
	m_roi_msg.pose.position.y = odm_ptr->pose.pose.position.y + complement_x*cos(pitch)*sin(yaw);
	m_roi_msg.pose.position.z = odm_ptr->pose.pose.position.z + complement_x*sin(pitch);

	m_roi_msg.pose.orientation.w = odm_ptr->pose.pose.orientation.w;
	m_roi_msg.pose.orientation.x = odm_ptr->pose.pose.orientation.x;
	m_roi_msg.pose.orientation.y = odm_ptr->pose.pose.orientation.y;
	m_roi_msg.pose.orientation.z = odm_ptr->pose.pose.orientation.z;

	m_roi_msg.scale.x = 2 * center;
	m_roi_msg.scale.y = 2 * m_roi_lateral_param;
	m_roi_msg.scale.z = 2 * m_roi_vertical_param;

	m_roi_box_pub.publish(m_roi_msg);
}

void ExtractLane::DesiredLocalWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr pose_array_ptr)
{
	if (!ExtractRegionOfInterest(pose_array_ptr)) ROS_ERROR_STREAM("Faile Extract ROI Lane");
	if (!Evaluation(pose_array_ptr)) ROS_ERROR_STREAM("Fail Evaluate");
}

bool ExtractLane::ExtractRegionOfInterest(const geometry_msgs::PoseArray::ConstPtr lane_ptr)
{
	m_roi_lane.header = lane_ptr->header;
    m_roi_lane.points.clear();

    for (auto pose : lane_ptr->poses){
		auto center = (m_roi_front_param + m_roi_rear_param)/2;

		auto thr_x = m_roi_msg.pose.position.x + center;
		auto thr_y = m_roi_msg.pose.position.y + m_roi_lateral_param;
		auto thr_z = m_roi_msg.pose.position.z + m_roi_vertical_param;
		if (((pose.position.x > m_roi_msg.pose.position.x - center) && (pose.position.x < m_roi_msg.pose.position.x + center)) &&
			((pose.position.y > m_roi_msg.pose.position.y - m_roi_lateral_param) && (pose.position.y < m_roi_msg.pose.position.y + m_roi_lateral_param)) &&
			((pose.position.z > m_roi_msg.pose.position.z - m_roi_vertical_param) && (pose.position.z < m_roi_msg.pose.position.z + m_roi_vertical_param))){
			m_roi_lane.points.push_back(pose.position);
		}			
	}
	if (m_roi_lane.points.size() < 1) return false;

	m_roi_lane_pub.publish(m_roi_lane);

	return true;
}

bool ExtractLane::Evaluation(const geometry_msgs::PoseArray::ConstPtr lane_ptr)
{
	auto diff_x = lane_ptr->poses.back().position.x - m_curr_uav_position.point.x;
	auto diff_y = lane_ptr->poses.back().position.y - m_curr_uav_position.point.y;
	auto diff_z = lane_ptr->poses.back().position.z - m_curr_uav_position.point.z;

	geometry_msgs::Point diff_msg;
	diff_msg.x = diff_x;
	diff_msg.y = diff_y;
	diff_msg.z = diff_z;

	m_evaulation_pub.publish(diff_msg);

	return true;
}

void ExtractLane::PolyfitLane ()
{
    
}