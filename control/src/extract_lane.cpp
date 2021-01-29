#include "extract_lane.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "uav_msgs/Roi.h"

ExtractLane::ExtractLane()
{
	// Set Param
	SetParam();

	// Topic name
	std::string uav_state_sub_topic_name = "/airsim_node/" + m_vehicle_name_param + "/odom_local_ned";

	// Initialize subscriber 
	m_desired_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>("/generate_waypoints_node/desired_waypoints", 10, boost::bind(&ExtractLane::DesiredWaypointsCallback, this, _1));
	m_uav_state_sub = m_nh.subscribe<nav_msgs::Odometry>(uav_state_sub_topic_name, 10, boost::bind(&ExtractLane::UavStateCallback, this, _1));

	// Initialize publisher 
	m_roi_box_pub = m_nh.advertise<uav_msgs::Roi> ("/extract_lane_node/roi", 1);
	m_roi_lane_pub = m_nh.advertise<uav_msgs::PolyfitLane> ("/extract_lane_node/poly_fit_lane", 1);
	// m_poly_fit_lane_pub = m_nh.advertise<uav_msgs::PolyfitLaneData> ("polyfit_lanes", 10);
}

ExtractLane::~ExtractLane()
{ }

void ExtractLane::SetParam()
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
	
	uav_msgs::Roi roi_msg;
	roi_msg.header = odm_ptr->header;

	auto euler = Quat2Euler(odm_ptr->pose.pose.orientation);
	auto roll = euler.r;
	auto pitch = euler.p;
	auto yaw = euler.y;
	roi_msg.pose.position.x = odm_ptr->pose.pose.position.x + complement_x*cos(pitch)*cos(yaw);
	roi_msg.pose.position.y = odm_ptr->pose.pose.position.y + complement_x*cos(pitch)*sin(yaw);
	roi_msg.pose.position.z = odm_ptr->pose.pose.position.z + complement_x*sin(pitch);

	roi_msg.pose.orientation.w = odm_ptr->pose.pose.orientation.w;
	roi_msg.pose.orientation.x = odm_ptr->pose.pose.orientation.x;
	roi_msg.pose.orientation.y = odm_ptr->pose.pose.orientation.y;
	roi_msg.pose.orientation.z = odm_ptr->pose.pose.orientation.z;

	roi_msg.scale.x = 2 * center;
	roi_msg.scale.y = 2 * m_roi_lateral_param;
	roi_msg.scale.z = 2 * m_roi_vertical_param;

	m_roi_box_pub.publish(roi_msg);
}

void ExtractLane::DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr pose_array_ptr)
{
	if(!ExtractRegionOfInterest(pose_array_ptr)) ROS_ERROR_STREAM("Faile Extract ROI Lane");
}

bool ExtractLane::ExtractRegionOfInterest(const geometry_msgs::PoseArray::ConstPtr lane_ptr)
{
	m_roi_lane.header = lane_ptr->header;
    m_roi_lane.points.clear();

    for (auto pose : lane_ptr->poses){
		double delta_x = m_curr_uav_position.point.x - pose.position.x;
		double delta_y = m_curr_uav_position.point.y - pose.position.y;
		double delta_z = m_curr_uav_position.point.z - pose.position.z;
		
		if ((delta_x <= m_roi_front_param) && 
			(delta_x >= -1*m_roi_rear_param) &&
			(std::fabs(delta_y) <= m_roi_lateral_param) &&
			(std::fabs(delta_z) <= m_roi_vertical_param)){
			
			m_roi_lane.points.push_back(pose.position);
		}
	}
	if (m_roi_lane.points.size() < 1) return false;

	m_roi_lane_pub.publish(m_roi_lane);
	return true;
}


void ExtractLane::PolyfitLane ()
{
    
}

Euler ExtractLane::Quat2Euler(const geometry_msgs::Quaternion& quat_msg)
{
	tf2::Quaternion quat_tf;
	double roll, pitch, yaw;
	tf2::fromMsg(quat_msg, quat_tf);
	tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
	
	Euler euler = {roll, pitch, yaw};

	return euler;
}