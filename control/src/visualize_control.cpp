#include "control/visualize_control.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

using namespace control;

VisualizeControl::VisualizeControl() :
    m_ugv_name_param("missing"),
    m_uav_name_param("missing")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();
    MarkerInit();
}

VisualizeControl::~VisualizeControl()
{}

void VisualizeControl::InitFlag()
{}

bool VisualizeControl::GetParam()
{
	m_nh.getParam("visualize_control_node/ugv_name", m_ugv_name_param);
	m_nh.getParam("visualize_control_node/uav_name", m_uav_name_param);
    
    if (m_ugv_name_param == "missing") { ROS_ERROR_STREAM("m_ugv_name_param is missing"); return false; }
    else if (m_uav_name_param == "missing") { ROS_ERROR_STREAM("m_uav_name_param is missing"); return false; }

    return true;
}

void VisualizeControl::InitROS()
{
    // package, node, topic name
    std::string node_name_with_namespace = ros::this_node::getName();
    std::string input_curr_position_sub_topic_name = "/airsim_node/" + m_ugv_name_param + "/car_state";
	std::string roi_curr_position_sub_topic_name = "/airsim_node/" + m_uav_name_param + "/odom_local_ned";

    // subscriber init
    m_ego_state_sub = m_nh.subscribe<uav_msgs::VehicleState>("/control/generate_waypoints_node/ego_state", 10, boost::bind(&VisualizeControl::EgoVehicleStateCallback, this, _1)); 
    m_target_state_sub = m_nh.subscribe<uav_msgs::VehicleStateArray>("/control/generate_waypoints_node/target_states", 10, boost::bind(&VisualizeControl::TargetVehicleStatesCallback, this, _1)); 
    m_target_waypoints_sub = m_nh.subscribe<uav_msgs::TargetWaypoints>("/control/generate_waypoints_node/target_waypoints", 1, boost::bind(&VisualizeControl::WaypointsCallback, this, _1));

    // m_roi_position_sub = m_nh.subscribe<nav_msgs::Odometry>(roi_curr_position_sub_topic_name, 1, boost::bind(&VisualizeControl::ROICurrPositionCallback, this, _1));
	// m_roi_waypoints_sub = m_nh.subscribe<uav_msgs::PolyfitLane>(self_pkg_name + "/extract_lane_node/poly_fit_lane", 1, boost::bind(&VisualizeControl::ROIWaypointsCallback, this, _1));
	// m_roi_box_sub = m_nh.subscribe<uav_msgs::Roi>(self_pkg_name + "/extract_lane_node/roi", 1, boost::bind(&VisualizeControl::ROICallback, this, _1));

    // publisher init
    m_markers_pub = m_nh.advertise<visualization_msgs::MarkerArray> (node_name_with_namespace + "/control_marekrs", 1);
    m_visualize_control_timer = m_nh.createTimer(ros::Duration(0.1), &VisualizeControl::TimerCallback, this);
}

void VisualizeControl::MarkerInit()
{
    //// ego trajectory
    m_ego_markers.line_strip.ns = "ego/trajectory";
    m_ego_markers.line_strip.id = 0;
    m_ego_markers.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_ego_markers.line_strip.action = visualization_msgs::Marker::ADD;
    m_ego_markers.line_strip.scale.x = 0.05;
    m_ego_markers.line_strip.pose.orientation.w = 1.0;
    m_ego_markers.line_strip.pose.orientation.x = 0.0;
    m_ego_markers.line_strip.pose.orientation.y = 0.0;
    m_ego_markers.line_strip.pose.orientation.z = 0.0;
    m_ego_markers.line_strip.color.r = 1.0f;
    m_ego_markers.line_strip.color.g = 1.0f;
    m_ego_markers.line_strip.color.b = 0.0f;
    m_ego_markers.line_strip.color.a = 0.4f;
    m_ego_markers.line_strip.lifetime = ros::Duration();
    m_ego_markers.is_line_strip_add = false;

    m_ego_markers.orientation.ns = "ego/orientation";
    m_ego_markers.orientation.id = 0;
    m_ego_markers.orientation.type = visualization_msgs::Marker::ARROW;
    m_ego_markers.orientation.action = visualization_msgs::Marker::ADD;
    m_ego_markers.orientation.scale.x = 2.0;
    m_ego_markers.orientation.scale.y = 0.1;
    m_ego_markers.orientation.scale.z = 0.1;
    m_ego_markers.orientation.color.r = 1.0f;
    m_ego_markers.orientation.color.g = 1.0f;
    m_ego_markers.orientation.color.b = 0.0f;
    m_ego_markers.orientation.color.a = 1.0f;
    m_ego_markers.orientation.lifetime = ros::Duration();
    m_ego_markers.is_orientation_add = false;

    m_ego_markers.center_point.ns = "ego/center_point";
    m_ego_markers.center_point.id = 0;
    m_ego_markers.center_point.type = visualization_msgs::Marker::SPHERE;
    m_ego_markers.center_point.action = visualization_msgs::Marker::ADD;
    m_ego_markers.center_point.scale.x = 0.3;
    m_ego_markers.center_point.scale.y = 0.3;
    m_ego_markers.center_point.scale.z = 0.3;
    m_ego_markers.center_point.pose.orientation.w = 1.0;
    m_ego_markers.center_point.pose.orientation.x = 0.0;
    m_ego_markers.center_point.pose.orientation.y = 0.0;
    m_ego_markers.center_point.pose.orientation.z = 0.0;
    m_ego_markers.center_point.color.r = 1.0f;
    m_ego_markers.center_point.color.g = 1.0f;
    m_ego_markers.center_point.color.b = 0.0f;
    m_ego_markers.center_point.color.a = 1.0f;
    m_ego_markers.center_point.lifetime = ros::Duration();
    m_ego_markers.is_center_point_add = false;
    
    m_ego_markers.txt.ns = "ego/txt";
    m_ego_markers.txt.id = 0;
    m_ego_markers.txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m_ego_markers.txt.action = visualization_msgs::Marker::ADD;
    m_ego_markers.txt.scale.z = 0.3;
    m_ego_markers.txt.pose.orientation.w = 1.0;
    m_ego_markers.txt.pose.orientation.x = 0.0;
    m_ego_markers.txt.pose.orientation.y = 0.0;
    m_ego_markers.txt.pose.orientation.z = 0.0;
    m_ego_markers.txt.color.r = 1.0f;
    m_ego_markers.txt.color.g = 1.0f;
    m_ego_markers.txt.color.b = 1.0f;
    m_ego_markers.txt.color.a = 1.0f;
    m_ego_markers.txt.lifetime = ros::Duration();
    m_ego_markers.is_txt_add = false;
    

    //// target markers
    for (int tool = (int)DetectionTool::AirSim ; tool < (int)DetectionTool::ItemNum; tool++){
        MetaMarkers target_markers;
        switch (tool){
            case (int)DetectionTool::AirSim:
            target_markers.line_strip.id = (int)DetectionTool::AirSim;
            target_markers.line_strip.color.r = 0.0f;
            target_markers.line_strip.color.g = 1.0f;
            target_markers.line_strip.color.b = 0.0f;
            target_markers.line_strip.color.a = 0.4f;

            target_markers.orientation.id = (int)DetectionTool::AirSim;
            target_markers.orientation.color.r = 0.0f;
            target_markers.orientation.color.g = 1.0f;
            target_markers.orientation.color.b = 0.0f;
            target_markers.orientation.color.a = 1.0f;

            target_markers.center_point.id = (int)DetectionTool::AirSim;
            target_markers.center_point.color.r = 0.0f;
            target_markers.center_point.color.g = 1.0f;
            target_markers.center_point.color.b = 0.0f;
            target_markers.center_point.color.a = 1.0f;

            target_markers.txt.id = (int)DetectionTool::AirSim;

            break;
            
            case (int)DetectionTool::LiDAR:
            target_markers.line_strip.id = (int)DetectionTool::LiDAR;
            target_markers.line_strip.color.r = 0.0f;
            target_markers.line_strip.color.g = 1.0f;
            target_markers.line_strip.color.b = 1.0f;
            target_markers.line_strip.color.a = 0.4f;

            target_markers.orientation.id = (int)DetectionTool::LiDAR;
            target_markers.orientation.color.r = 0.0f;
            target_markers.orientation.color.g = 1.0f;
            target_markers.orientation.color.b = 1.0f;
            target_markers.orientation.color.a = 1.0f;

            target_markers.center_point.id = (int)DetectionTool::LiDAR;
            target_markers.center_point.color.r = 0.0f;
            target_markers.center_point.color.g = 1.0f;
            target_markers.center_point.color.b = 1.0f;
            target_markers.center_point.color.a = 1.0f;

            target_markers.txt.id = (int)DetectionTool::LiDAR;

            break;

            case (int)DetectionTool::GPS:
            target_markers.line_strip.id = (int)DetectionTool::GPS;
            target_markers.line_strip.color.r = 1.0f;
            target_markers.line_strip.color.g = 0.0f;
            target_markers.line_strip.color.b = 1.0f;
            target_markers.line_strip.color.a = 0.4f;

            target_markers.orientation.id = (int)DetectionTool::GPS;
            target_markers.orientation.color.r = 1.0f;
            target_markers.orientation.color.g = 0.0f;
            target_markers.orientation.color.b = 1.0f;
            target_markers.orientation.color.a = 1.0f;

            target_markers.center_point.id = (int)DetectionTool::GPS;
            target_markers.center_point.color.r = 1.0f;
            target_markers.center_point.color.g = 0.0f;
            target_markers.center_point.color.b = 1.0f;
            target_markers.center_point.color.a = 1.0f;

            target_markers.txt.id = (int)DetectionTool::GPS;

            break;
            
            default:
            break;
        }
    
        target_markers.line_strip.ns = "target/trajectory";
        target_markers.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        target_markers.line_strip.action = visualization_msgs::Marker::ADD;
        target_markers.line_strip.scale.x = 0.05;
        target_markers.line_strip.pose.orientation.w = 1.0;
        target_markers.line_strip.pose.orientation.x = 0.0;
        target_markers.line_strip.pose.orientation.y = 0.0;
        target_markers.line_strip.pose.orientation.z = 0.0;
        target_markers.line_strip.lifetime = ros::Duration();
        target_markers.is_line_strip_add = false;

        target_markers.orientation.ns = "target/orientation";
        target_markers.orientation.type = visualization_msgs::Marker::ARROW;
        target_markers.orientation.action = visualization_msgs::Marker::ADD;
        target_markers.orientation.scale.x = 2.0;
        target_markers.orientation.scale.y = 0.1;
        target_markers.orientation.scale.z = 0.1;
        target_markers.orientation.lifetime = ros::Duration();
        target_markers.is_orientation_add = false;

        target_markers.center_point.ns = "target/center_point";
        target_markers.center_point.type = visualization_msgs::Marker::SPHERE;
        target_markers.center_point.action = visualization_msgs::Marker::ADD;
        target_markers.center_point.scale.x = 0.3;
        target_markers.center_point.scale.y = 0.3;
        target_markers.center_point.scale.z = 0.3;
        target_markers.center_point.pose.orientation.w = 1.0;
        target_markers.center_point.pose.orientation.x = 0.0;
        target_markers.center_point.pose.orientation.y = 0.0;
        target_markers.center_point.pose.orientation.z = 0.0;
        target_markers.center_point.lifetime = ros::Duration();
        target_markers.is_center_point_add = false;

        target_markers.txt.ns = "target/txt";
        target_markers.txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        target_markers.txt.action = visualization_msgs::Marker::ADD;
        target_markers.txt.scale.z = 0.3;
        target_markers.txt.pose.orientation.w = 1.0;
        target_markers.txt.pose.orientation.x = 0.0;
        target_markers.txt.pose.orientation.y = 0.0;
        target_markers.txt.pose.orientation.z = 0.0;
        target_markers.txt.color.r = 1.0f;
        target_markers.txt.color.g = 1.0f;
        target_markers.txt.color.b = 1.0f;
        target_markers.txt.color.a = 1.0f;
        target_markers.txt.lifetime = ros::Duration();
        target_markers.is_txt_add = false;

        m_targets_markers.push_back(target_markers);
    }

    //// waypoints
    m_waypoints_markers.line_strip.ns = "waypoints/trajectory";
    m_waypoints_markers.line_strip.id = 0;
    m_waypoints_markers.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_waypoints_markers.line_strip.action = visualization_msgs::Marker::ADD;
    m_waypoints_markers.line_strip.scale.x = 0.05;
    m_waypoints_markers.line_strip.pose.orientation.w = 1.0;
    m_waypoints_markers.line_strip.pose.orientation.x = 0.0;
    m_waypoints_markers.line_strip.pose.orientation.y = 0.0;
    m_waypoints_markers.line_strip.pose.orientation.z = 0.0;
    m_waypoints_markers.line_strip.color.r = 1.0f;
    m_waypoints_markers.line_strip.color.g = 0.0f;
    m_waypoints_markers.line_strip.color.b = 0.0f;
    m_waypoints_markers.line_strip.color.a = 0.4f;
    m_waypoints_markers.line_strip.lifetime = ros::Duration();
    m_waypoints_markers.is_line_strip_add = false;


    //// roi line
    m_roi_markers.line_strip.ns = "roi_line/trajectory";
    m_roi_markers.line_strip.id = 0;
    m_roi_markers.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_roi_markers.line_strip.action = visualization_msgs::Marker::ADD;
    m_roi_markers.line_strip.scale.x = 0.1;
    m_roi_markers.line_strip.pose.orientation.w = 1.0;
    m_roi_markers.line_strip.pose.orientation.x = 0.0;
    m_roi_markers.line_strip.pose.orientation.y = 0.0;
    m_roi_markers.line_strip.pose.orientation.z = 0.0;
    m_roi_markers.line_strip.color.r = 0.5f;
    m_roi_markers.line_strip.color.g = 0.5f;
    m_roi_markers.line_strip.color.b = 0.0f;
    m_roi_markers.line_strip.color.a = 0.4f;
    m_roi_markers.line_strip.lifetime = ros::Duration();
    m_roi_markers.is_line_strip_add = false;

    m_roi_markers.roi.ns = "roi_line/roi";
    m_roi_markers.roi.id = 0;
    m_roi_markers.roi.type = visualization_msgs::Marker::CUBE;
    m_roi_markers.roi.action = visualization_msgs::Marker::ADD;
    m_roi_markers.roi.color.r = 1.0f;
    m_roi_markers.roi.color.g = 0.0f;
    m_roi_markers.roi.color.b = 1.0f;
    m_roi_markers.roi.color.a = 0.3f;
    m_roi_markers.roi.lifetime = ros::Duration();
    m_roi_markers.is_roi_add = false;
}


void VisualizeControl::TimerCallback(const ros::TimerEvent& event)
{
    m_ego_markers.self.markers.clear();
    if (m_ego_markers.is_line_strip_add) m_ego_markers.self.markers.push_back(m_ego_markers.line_strip);
    if (m_ego_markers.is_orientation_add) m_ego_markers.self.markers.push_back(m_ego_markers.orientation);
    if (m_ego_markers.is_center_point_add) m_ego_markers.self.markers.push_back(m_ego_markers.center_point);
    if (m_ego_markers.is_txt_add) m_ego_markers.self.markers.push_back(m_ego_markers.txt);
    m_ego_markers.is_line_strip_add = m_ego_markers.is_orientation_add = m_ego_markers.is_center_point_add = m_ego_markers.is_txt_add = false;

    for (auto &target_markers : m_targets_markers){
        target_markers.self.markers.clear();
        if (target_markers.is_line_strip_add) target_markers.self.markers.push_back(target_markers.line_strip);
        if (target_markers.is_orientation_add) target_markers.self.markers.push_back(target_markers.orientation);
        if (target_markers.is_center_point_add) target_markers.self.markers.push_back(target_markers.center_point);
        if (target_markers.is_txt_add) target_markers.self.markers.push_back(target_markers.txt);
        target_markers.is_line_strip_add = target_markers.is_orientation_add = target_markers.is_center_point_add = target_markers.is_txt_add = false;;
    }

    m_waypoints_markers.self.markers.clear();
    if (m_waypoints_markers.is_line_strip_add) m_waypoints_markers.self.markers.push_back(m_waypoints_markers.line_strip);
    m_waypoints_markers.is_line_strip_add = false;

    m_roi_markers.self.markers.clear();
    if (m_roi_markers.is_line_strip_add) m_roi_markers.self.markers.push_back(m_roi_markers.line_strip);
    if (m_roi_markers.is_center_point_add) m_roi_markers.self.markers.push_back(m_roi_markers.center_point);
    if (m_roi_markers.is_roi_add) m_roi_markers.self.markers.push_back(m_roi_markers.roi);
    m_roi_markers.is_line_strip_add = m_roi_markers.is_center_point_add  = m_roi_markers.is_roi_add = false;

    visualization_msgs::MarkerArray visualization_markers;
    visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                        m_ego_markers.self.markers.begin(), m_ego_markers.self.markers.end());
    for (auto &target_markers : m_targets_markers){
        visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                            target_markers.self.markers.begin(), target_markers.self.markers.end());                                        
    }
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        m_waypoints_markers.self.markers.begin(), m_waypoints_markers.self.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        m_roi_markers.self.markers.begin(), m_roi_markers.self.markers.end());

    m_markers_pub.publish(visualization_markers);
}


void VisualizeControl::EgoVehicleStateCallback(const uav_msgs::VehicleState::ConstPtr &ego_vehicle_ptr)
{
    // center_point
    m_ego_markers.center_point.header.frame_id = ego_vehicle_ptr->local_posestamped.header.frame_id;
    m_ego_markers.center_point.header.stamp = ros::Time(0);

    m_ego_markers.center_point.pose = ego_vehicle_ptr->local_posestamped.pose;
    m_ego_markers.is_center_point_add = true;

    // orientation
    double vehicle_speed_ms = m_utils.Size(ego_vehicle_ptr->velocity.linear.x, ego_vehicle_ptr->velocity.linear.y, ego_vehicle_ptr->velocity.linear.z);
    double vehicle_speed_kmh = m_utils.ms_to_kmh(vehicle_speed_ms);
    double arrow_size = 2.0*m_utils.VelNomalize(vehicle_speed_kmh);
    m_ego_markers.orientation.header.frame_id = ego_vehicle_ptr->local_posestamped.header.frame_id;
    m_ego_markers.orientation.header.stamp = ros::Time(0);
    
    m_ego_markers.orientation.scale.x = arrow_size;
    m_ego_markers.orientation.pose = ego_vehicle_ptr->local_posestamped.pose;
    m_ego_markers.is_orientation_add = true;

    // txt
    m_ego_markers.txt.header.frame_id = ego_vehicle_ptr->local_posestamped.header.frame_id;
    m_ego_markers.txt.header.stamp = ros::Time(0);

    m_ego_markers.txt.pose.position = ego_vehicle_ptr->local_posestamped.pose.position;
    m_ego_markers.txt.pose.position.z += 0.8;
    m_ego_markers.txt.text = "v_xyz: " + m_utils.ToString(vehicle_speed_kmh) + " [kmh]\np_z: " + m_utils.ToString(ego_vehicle_ptr->local_posestamped.pose.position.z) + " [m]";
    m_ego_markers.is_txt_add = true;


    // line strip
    m_ego_markers.line_strip.header.frame_id = ego_vehicle_ptr->local_posestamped.header.frame_id;
    m_ego_markers.line_strip.header.stamp = ros::Time(0);
    
    m_ego_markers.line_strip.points.clear();
    for (auto p : ego_vehicle_ptr->local_trajectory.poses){
        m_ego_markers.line_strip.points.push_back(p.position);
    }
    m_ego_markers.is_line_strip_add = true;

}

void VisualizeControl::TargetVehicleStatesCallback(const uav_msgs::VehicleStateArray::ConstPtr &target_vehicles_ptr)
{
    for (int tool = (int)DetectionTool::AirSim; tool < (int)DetectionTool::ItemNum; tool++){
        if (target_vehicles_ptr->vehicle_states[tool].local_trajectory.poses.size() != 0){
            // center_point
            m_targets_markers[tool].center_point.header.frame_id = target_vehicles_ptr->vehicle_states[tool].local_posestamped.header.frame_id;
            m_targets_markers[tool].center_point.header.stamp = ros::Time(0);

            m_targets_markers[tool].center_point.pose = target_vehicles_ptr->vehicle_states[tool].local_posestamped.pose;
            m_targets_markers[tool].is_center_point_add = true;

            // orientation
            double vehicle_speed_ms = m_utils.Size(target_vehicles_ptr->vehicle_states[tool].velocity.linear.x, target_vehicles_ptr->vehicle_states[tool].velocity.linear.y, target_vehicles_ptr->vehicle_states[tool].velocity.linear.z);
            double vehicle_speed_kmh = m_utils.ms_to_kmh(vehicle_speed_ms);
            double arrow_size = 2.0*m_utils.VelNomalize(vehicle_speed_kmh);
            m_targets_markers[tool].orientation.header.frame_id = target_vehicles_ptr->vehicle_states[tool].local_posestamped.header.frame_id;
            m_targets_markers[tool].orientation.header.stamp = ros::Time(0);
            
            m_targets_markers[tool].orientation.scale.x = arrow_size;
            m_targets_markers[tool].orientation.pose = target_vehicles_ptr->vehicle_states[tool].local_posestamped.pose;
            m_targets_markers[tool].is_orientation_add = true;

            // txt
            m_targets_markers[tool].txt.header.frame_id = target_vehicles_ptr->vehicle_states[tool].local_posestamped.header.frame_id;
            m_targets_markers[tool].txt.header.stamp = ros::Time(0);

            m_targets_markers[tool].txt.pose.position = target_vehicles_ptr->vehicle_states[tool].local_posestamped.pose.position;
            m_targets_markers[tool].txt.pose.position.z += 0.8;
            m_targets_markers[tool].txt.text 
                = "v_xyz: " + m_utils.ToString(vehicle_speed_kmh) + " [kmh]\np_z: " + m_utils.ToString(target_vehicles_ptr->vehicle_states[tool].local_posestamped.pose.position.z) + " [m]";
            m_targets_markers[tool].is_txt_add = true;


            // line strip
            m_targets_markers[tool].line_strip.header.frame_id = target_vehicles_ptr->vehicle_states[tool].local_posestamped.header.frame_id;
            m_targets_markers[tool].line_strip.header.stamp = ros::Time(0);
            
            m_targets_markers[tool].line_strip.points.clear();
            for (auto p : target_vehicles_ptr->vehicle_states[tool].local_trajectory.poses){
                m_targets_markers[tool].line_strip.points.push_back(p.position);
            }
            m_targets_markers[tool].is_line_strip_add = true;
        }
    }
}

void VisualizeControl::WaypointsCallback(const uav_msgs::TargetWaypoints::ConstPtr &target_wp_ptr)
{
    // line_strip
    m_waypoints_markers.line_strip.header.frame_id = target_wp_ptr->local.header.frame_id;
    m_waypoints_markers.line_strip.header.stamp = ros::Time(0);

    m_waypoints_markers.line_strip.points.clear();
    for (auto p : target_wp_ptr->local.poses){
        m_waypoints_markers.line_strip.points.push_back(p.position);
    }
    m_waypoints_markers.is_line_strip_add = true;
}


void VisualizeControl::ROIWaypointsCallback(const uav_msgs::PolyfitLane::ConstPtr &roi_waypoints_ptr)
{
    // line_strip
    m_roi_markers.line_strip.header.frame_id = roi_waypoints_ptr->header.frame_id;
    m_roi_markers.line_strip.header.stamp = ros::Time(0);

    m_roi_markers.line_strip.points.clear();
    for (auto p : roi_waypoints_ptr->points){
        m_roi_markers.line_strip.points.push_back(p);
    }
    m_roi_markers.is_line_strip_add = true;
}

void VisualizeControl::ROICurrPositionCallback(const nav_msgs::Odometry::ConstPtr &current_point_ptr)
{
    // center_point
    m_roi_markers.center_point.header.frame_id = current_point_ptr->header.frame_id;
    m_roi_markers.center_point.header.stamp = ros::Time(0);

    m_roi_markers.center_point.pose.position.x = current_point_ptr->pose.pose.position.x;
    m_roi_markers.center_point.pose.position.y = current_point_ptr->pose.pose.position.y;
    m_roi_markers.center_point.pose.position.z = current_point_ptr->pose.pose.position.z;
    m_roi_markers.is_center_point_add = true;
}

void VisualizeControl::ROICallback(const uav_msgs::Roi::ConstPtr &roi_ptr)
{
    // roi
    m_roi_markers.roi.header.frame_id = roi_ptr->header.frame_id;
    m_roi_markers.roi.header.stamp = ros::Time(0);

    m_roi_markers.roi.pose.position.x = roi_ptr->pose.position.x;
    m_roi_markers.roi.pose.position.y = roi_ptr->pose.position.y;
    m_roi_markers.roi.pose.position.z = roi_ptr->pose.position.z;

    m_roi_markers.roi.pose.orientation.w = roi_ptr->pose.orientation.w;
    m_roi_markers.roi.pose.orientation.x = roi_ptr->pose.orientation.x;
    m_roi_markers.roi.pose.orientation.y = roi_ptr->pose.orientation.y;
    m_roi_markers.roi.pose.orientation.z = roi_ptr->pose.orientation.z;

    m_roi_markers.roi.scale.x = roi_ptr->scale.x;
    m_roi_markers.roi.scale.y = roi_ptr->scale.y;
    m_roi_markers.roi.scale.z = roi_ptr->scale.z;
    m_roi_markers.is_roi_add = true;
}