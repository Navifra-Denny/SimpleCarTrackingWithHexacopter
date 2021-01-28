#include "extract_lane.h"

ExtractLane::ExtractLane()
{
	// Set Param
	SetParam();

	// Initialize subscriber 
	m_desired_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>("/generate_waypoints_node/desired_waypoints", 10, boost::bind(&ExtractLane::DesiredWaypointsCallback, this, _1));

	std::string m_uav_state_sub_topic_name = "/airsim_node/" + m_vehicle_name_param + "/odom_local_ned";
	m_uav_state_sub = m_nh.subscribe<nav_msgs::Odometry>(m_uav_state_sub_topic_name, 10, boost::bind(&ExtractLane::UavStateCallback, this, _1));

	// Initialize publisher 
	m_roi_lane_pub = m_nh.advertise<uav_msgs::PolyfitLane> ("/extract_lane_node/poly_fit_lane", 1);
	// m_poly_fit_lane_pub = m_nh.advertise<uav_msgs::PolyfitLaneData> ("polyfit_lanes", 10);
	m_curr_position_pub = m_nh.advertise<geometry_msgs::Point> ("/extract_lane_node/current_point", 10);
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
    auto position = odm_ptr->pose.pose.position;
    
    m_curr_uav_position.x = position.x;
    m_curr_uav_position.y = position.y;
    m_curr_uav_position.z = position.z;
	
	geometry_msgs::Point curr_p = m_curr_uav_position;
	m_curr_position_pub.publish(curr_p);
}

void ExtractLane::DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr pose_array_ptr)
{
    std::vector<geometry_msgs::Point> lane = GetLane(pose_array_ptr);
    if(!ExtractRegionOfInterest(lane)) ROS_ERROR_STREAM("Faile Extract ROI Lane");
}

bool ExtractLane::ExtractRegionOfInterest(std::vector<geometry_msgs::Point> input_lane)
{
    m_roi_lane.header.frame_id = "world_ned";
    m_roi_lane.points.clear();

    for (auto p : input_lane){
		double delta_x = m_curr_uav_position.x - p.x;
		double delta_y = m_curr_uav_position.y - p.y;
		double delta_z = m_curr_uav_position.z - p.z;
		
		if ((delta_x <= m_roi_front_param) && 
			(delta_x >= -1*m_roi_rear_param) &&
			(std::fabs(delta_y) <= m_roi_lateral_param) &&
			(std::fabs(delta_z) <= m_roi_vertical_param)){
			
			m_roi_lane.points.push_back(p);
		}
	}
	if (m_roi_lane.points.size() < 1) return false;

	m_roi_lane_pub.publish(m_roi_lane);
	return true;
}


void ExtractLane::PolyfitLane ()
{
    
}

std::vector<geometry_msgs::Point> ExtractLane::GetLane(const geometry_msgs::PoseArray::ConstPtr pose_array_ptr)
{
    std::vector<geometry_msgs::Point> lane;

    for (auto const p : pose_array_ptr->poses){
        geometry_msgs::Point point;

        point.x = p.position.x;
        point.y = p.position.y;
        point.z = p.position.z;
        
        lane.push_back(point);
    }

    return lane;
}