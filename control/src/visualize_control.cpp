#include "control/visualize_control.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

VisualizeControl::VisualizeControl() :
    m_ugv_name_param("missing"),
    m_uav_name_param("missing"),
    m_target_height_m_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();
    MarkerInit();
}

VisualizeControl::~VisualizeControl()
{}

void VisualizeControl::InitFlag()
{
    m_is_home_set = false;
}

bool VisualizeControl::GetParam()
{
	m_nh.getParam("visualize_control_node/ugv_name", m_ugv_name_param);
	m_nh.getParam("visualize_control_node/uav_name", m_uav_name_param);
	m_nh.getParam("visualize_control_node/target_height_m", m_target_height_m_param);
    
    if (m_ugv_name_param == "missing") { ROS_ERROR_STREAM("m_ugv_name_param is missing"); return false; }
    else if (m_uav_name_param == "missing") { ROS_ERROR_STREAM("m_uav_name_param is missing"); return false; }
    else if (__isnan(m_target_height_m_param)) { ROS_ERROR_STREAM("m_target_height_m_param is NAN"); return false; }

    return true;
}

void VisualizeControl::InitROS()
{
    // package, node, topic name
    std::string self_pkg_name = "/control";
    std::string self_node_name = "/visualzie_control_node";
    std::string input_curr_position_sub_topic_name = "/airsim_node/" + m_ugv_name_param + "/car_state";
	std::string roi_curr_position_sub_topic_name = "/airsim_node/" + m_uav_name_param + "/odom_local_ned";

    // subscriber init
    m_ego_vehicle_local_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&VisualizeControl::EgoVehicleLocalPositionCallback, this, _1));
    m_ego_vehicle_local_vel_sub = m_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, boost::bind(&VisualizeControl::EgoVehicleLocalVelocityCallback, this, _1));
    m_ego_trajectory_sub = m_nh.subscribe<uav_msgs::Trajectory>(self_pkg_name + "/generate_waypoints_node/ego_trajectory", 1, boost::bind(&VisualizeControl::EgoTrajectoryCallback, this, _1));
    
    m_target_vehicle_global_position_sub = 
        m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&VisualizeControl::TargetVehicleGlobalStateCallback, this, _1));
	m_target_vehicle_local_ned_position_sub = m_nh.subscribe<uav_msgs::CarState>(input_curr_position_sub_topic_name, 10, boost::bind(&VisualizeControl::TargetVehicleLocalNedPositionCallback, this, _1));
    m_target_trajectory_sub = m_nh.subscribe<uav_msgs::Trajectory>(self_pkg_name + "/generate_waypoints_node/target_trajectory", 1, boost::bind(&VisualizeControl::TargetTrajectoryCallback, this, _1));
	
    m_target_waypoints_sub = m_nh.subscribe<uav_msgs::TargetWP>(self_pkg_name + "/generate_waypoints_node/target_waypoints", 1, boost::bind(&VisualizeControl::WaypointsCallback, this, _1));
    m_home_position_sub = 
        m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&VisualizeControl::HomePositionCallback, this, _1));
    ros::Rate rate(10);
    while (ros::ok() && !m_is_home_set){
        ros::spinOnce();
        rate.sleep();
    }

    // m_roi_position_sub = m_nh.subscribe<nav_msgs::Odometry>(roi_curr_position_sub_topic_name, 1, boost::bind(&VisualizeControl::ROICurrPositionCallback, this, _1));
	// m_roi_waypoints_sub = m_nh.subscribe<uav_msgs::PolyfitLane>(self_pkg_name + "/extract_lane_node/poly_fit_lane", 1, boost::bind(&VisualizeControl::ROIWaypointsCallback, this, _1));
	// m_roi_box_sub = m_nh.subscribe<uav_msgs::Roi>(self_pkg_name + "/extract_lane_node/roi", 1, boost::bind(&VisualizeControl::ROICallback, this, _1));

    // publisher init
    m_markers_pub = m_nh.advertise<visualization_msgs::MarkerArray> (self_pkg_name + self_node_name + "/control_marekrs", 1);
    m_roi_line_update_timer = m_nh.createTimer(ros::Duration(0.1), &VisualizeControl::TimerCallback, this);
}

void VisualizeControl::MarkerInit()
{
    //// target trajectory
    m_target_trajectory.line_strip.ns = "target_trajectory/lane";
    m_target_trajectory.line_strip.id = 0;
    m_target_trajectory.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_target_trajectory.line_strip.action = visualization_msgs::Marker::ADD;
    m_target_trajectory.line_strip.scale.x = 0.05;
    m_target_trajectory.line_strip.pose.orientation.w = 1.0;
    m_target_trajectory.line_strip.pose.orientation.x = 0.0;
    m_target_trajectory.line_strip.pose.orientation.y = 0.0;
    m_target_trajectory.line_strip.pose.orientation.z = 0.0;
    m_target_trajectory.line_strip.color.r = 0.0f;
    m_target_trajectory.line_strip.color.g = 1.0f;
    m_target_trajectory.line_strip.color.b = 0.0f;
    m_target_trajectory.line_strip.color.a = 0.4f;
    m_target_trajectory.line_strip.lifetime = ros::Duration();
    m_target_trajectory.is_line_strip_add = false;

    m_target_trajectory.orientation.ns = "target_trajectory/orientation";
    m_target_trajectory.orientation.id = 0;
    m_target_trajectory.orientation.type = visualization_msgs::Marker::ARROW;
    m_target_trajectory.orientation.action = visualization_msgs::Marker::ADD;
    m_target_trajectory.orientation.scale.x = 2.0;
    m_target_trajectory.orientation.scale.y = 0.1;
    m_target_trajectory.orientation.scale.z = 0.1;
    m_target_trajectory.orientation.color.r = 0.0f;
    m_target_trajectory.orientation.color.g = 1.0f;
    m_target_trajectory.orientation.color.b = 0.0f;
    m_target_trajectory.orientation.color.a = 1.0f;
    m_target_trajectory.orientation.lifetime = ros::Duration();
    m_target_trajectory.is_orientation_add = false;

    m_target_trajectory.center_point.ns = "target_trajectory/center_point";
    m_target_trajectory.center_point.id = 0;
    m_target_trajectory.center_point.type = visualization_msgs::Marker::SPHERE;
    m_target_trajectory.center_point.action = visualization_msgs::Marker::ADD;
    m_target_trajectory.center_point.scale.x = 0.3;
    m_target_trajectory.center_point.scale.y = 0.3;
    m_target_trajectory.center_point.scale.z = 0.3;
    m_target_trajectory.center_point.pose.orientation.w = 1.0;
    m_target_trajectory.center_point.pose.orientation.x = 0.0;
    m_target_trajectory.center_point.pose.orientation.y = 0.0;
    m_target_trajectory.center_point.pose.orientation.z = 0.0;
    m_target_trajectory.center_point.color.r = 0.0f;
    m_target_trajectory.center_point.color.g = 1.0f;
    m_target_trajectory.center_point.color.b = 0.0f;
    m_target_trajectory.center_point.color.a = 1.0f;
    m_target_trajectory.center_point.lifetime = ros::Duration();
    m_target_trajectory.is_center_point_add = false;
    
    m_target_trajectory.txt.ns = "m_target_trajectory/txt";
    m_target_trajectory.txt.id = 0;
    m_target_trajectory.txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m_target_trajectory.txt.action = visualization_msgs::Marker::ADD;
    m_target_trajectory.txt.scale.z = 0.3;
    m_target_trajectory.txt.pose.orientation.w = 1.0;
    m_target_trajectory.txt.pose.orientation.x = 0.0;
    m_target_trajectory.txt.pose.orientation.y = 0.0;
    m_target_trajectory.txt.pose.orientation.z = 0.0;
    m_target_trajectory.txt.color.r = 1.0f;
    m_target_trajectory.txt.color.g = 1.0f;
    m_target_trajectory.txt.color.b = 1.0f;
    m_target_trajectory.txt.color.a = 1.0f;
    m_target_trajectory.txt.lifetime = ros::Duration();
    m_target_trajectory.is_txt_add = false;

    //// ego trajectory
    m_ego_trajectory.line_strip.ns = "ego_trajectory/lane";
    m_ego_trajectory.line_strip.id = 0;
    m_ego_trajectory.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_ego_trajectory.line_strip.action = visualization_msgs::Marker::ADD;
    m_ego_trajectory.line_strip.scale.x = 0.05;
    m_ego_trajectory.line_strip.pose.orientation.w = 1.0;
    m_ego_trajectory.line_strip.pose.orientation.x = 0.0;
    m_ego_trajectory.line_strip.pose.orientation.y = 0.0;
    m_ego_trajectory.line_strip.pose.orientation.z = 0.0;
    m_ego_trajectory.line_strip.color.r = 1.0f;
    m_ego_trajectory.line_strip.color.g = 1.0f;
    m_ego_trajectory.line_strip.color.b = 0.0f;
    m_ego_trajectory.line_strip.color.a = 0.4f;
    m_ego_trajectory.line_strip.lifetime = ros::Duration();
    m_ego_trajectory.is_line_strip_add = false;

    m_ego_trajectory.orientation.ns = "ego_trajectory/orientation";
    m_ego_trajectory.orientation.id = 0;
    m_ego_trajectory.orientation.type = visualization_msgs::Marker::ARROW;
    m_ego_trajectory.orientation.action = visualization_msgs::Marker::ADD;
    m_ego_trajectory.orientation.scale.x = 2.0;
    m_ego_trajectory.orientation.scale.y = 0.1;
    m_ego_trajectory.orientation.scale.z = 0.1;
    m_ego_trajectory.orientation.color.r = 1.0f;
    m_ego_trajectory.orientation.color.g = 1.0f;
    m_ego_trajectory.orientation.color.b = 0.0f;
    m_ego_trajectory.orientation.color.a = 1.0f;
    m_ego_trajectory.orientation.lifetime = ros::Duration();
    m_ego_trajectory.is_orientation_add = false;

    m_ego_trajectory.center_point.ns = "ego_trajectory/center_point";
    m_ego_trajectory.center_point.id = 0;
    m_ego_trajectory.center_point.type = visualization_msgs::Marker::SPHERE;
    m_ego_trajectory.center_point.action = visualization_msgs::Marker::ADD;
    m_ego_trajectory.center_point.scale.x = 0.3;
    m_ego_trajectory.center_point.scale.y = 0.3;
    m_ego_trajectory.center_point.scale.z = 0.3;
    m_ego_trajectory.center_point.pose.orientation.w = 1.0;
    m_ego_trajectory.center_point.pose.orientation.x = 0.0;
    m_ego_trajectory.center_point.pose.orientation.y = 0.0;
    m_ego_trajectory.center_point.pose.orientation.z = 0.0;
    m_ego_trajectory.center_point.color.r = 1.0f;
    m_ego_trajectory.center_point.color.g = 1.0f;
    m_ego_trajectory.center_point.color.b = 0.0f;
    m_ego_trajectory.center_point.color.a = 1.0f;
    m_ego_trajectory.center_point.lifetime = ros::Duration();
    m_ego_trajectory.is_center_point_add = false;
    
    m_ego_trajectory.txt.ns = "ego_trajectory/txt";
    m_ego_trajectory.txt.id = 0;
    m_ego_trajectory.txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m_ego_trajectory.txt.action = visualization_msgs::Marker::ADD;
    m_ego_trajectory.txt.scale.z = 0.3;
    m_ego_trajectory.txt.pose.orientation.w = 1.0;
    m_ego_trajectory.txt.pose.orientation.x = 0.0;
    m_ego_trajectory.txt.pose.orientation.y = 0.0;
    m_ego_trajectory.txt.pose.orientation.z = 0.0;
    m_ego_trajectory.txt.color.r = 1.0f;
    m_ego_trajectory.txt.color.g = 1.0f;
    m_ego_trajectory.txt.color.b = 1.0f;
    m_ego_trajectory.txt.color.a = 1.0f;
    m_ego_trajectory.txt.lifetime = ros::Duration();
    m_ego_trajectory.is_txt_add = false;
    

    //// waypoints
    m_target_waypoints.line_strip.ns = "waypoints/lane";
    m_target_waypoints.line_strip.id = 0;
    m_target_waypoints.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_target_waypoints.line_strip.action = visualization_msgs::Marker::ADD;
    m_target_waypoints.line_strip.scale.x = 0.05;
    m_target_waypoints.line_strip.pose.orientation.w = 1.0;
    m_target_waypoints.line_strip.pose.orientation.x = 0.0;
    m_target_waypoints.line_strip.pose.orientation.y = 0.0;
    m_target_waypoints.line_strip.pose.orientation.z = 0.0;
    m_target_waypoints.line_strip.color.r = 1.0f;
    m_target_waypoints.line_strip.color.g = 0.0f;
    m_target_waypoints.line_strip.color.b = 0.0f;
    m_target_waypoints.line_strip.color.a = 0.4f;
    m_target_waypoints.line_strip.lifetime = ros::Duration();
    m_target_waypoints.is_line_strip_add = false;


    //// roi line
    m_roi_line.line_strip.ns = "roi_line/lane";
    m_roi_line.line_strip.id = 0;
    m_roi_line.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_roi_line.line_strip.action = visualization_msgs::Marker::ADD;
    m_roi_line.line_strip.scale.x = 0.1;
    m_roi_line.line_strip.pose.orientation.w = 1.0;
    m_roi_line.line_strip.pose.orientation.x = 0.0;
    m_roi_line.line_strip.pose.orientation.y = 0.0;
    m_roi_line.line_strip.pose.orientation.z = 0.0;
    m_roi_line.line_strip.color.r = 0.5f;
    m_roi_line.line_strip.color.g = 0.5f;
    m_roi_line.line_strip.color.b = 0.0f;
    m_roi_line.line_strip.color.a = 0.4f;
    m_roi_line.line_strip.lifetime = ros::Duration();
    m_roi_line.is_line_strip_add = false;

    m_roi_line.roi.ns = "roi_line/roi";
    m_roi_line.roi.id = 0;
    m_roi_line.roi.type = visualization_msgs::Marker::CUBE;
    m_roi_line.roi.action = visualization_msgs::Marker::ADD;
    m_roi_line.roi.color.r = 1.0f;
    m_roi_line.roi.color.g = 0.0f;
    m_roi_line.roi.color.b = 1.0f;
    m_roi_line.roi.color.a = 0.3f;
    m_roi_line.roi.lifetime = ros::Duration();
    m_roi_line.is_roi_add = false;
}


void VisualizeControl::TimerCallback(const ros::TimerEvent& event)
{
    m_ego_trajectory.self.markers.clear();
    if (m_ego_trajectory.is_line_strip_add) m_ego_trajectory.self.markers.push_back(m_ego_trajectory.line_strip);
    if (m_ego_trajectory.is_orientation_add) m_ego_trajectory.self.markers.push_back(m_ego_trajectory.orientation);
    if (m_ego_trajectory.is_center_point_add) m_ego_trajectory.self.markers.push_back(m_ego_trajectory.center_point);
    if (m_ego_trajectory.is_txt_add) m_ego_trajectory.self.markers.push_back(m_ego_trajectory.txt);
    m_ego_trajectory.is_line_strip_add = m_ego_trajectory.is_orientation_add = m_ego_trajectory.is_center_point_add = m_ego_trajectory.is_txt_add = false;

    m_target_trajectory.self.markers.clear();
    if (m_target_trajectory.is_line_strip_add) m_target_trajectory.self.markers.push_back(m_target_trajectory.line_strip);
    if (m_target_trajectory.is_orientation_add) m_target_trajectory.self.markers.push_back(m_target_trajectory.orientation);
    if (m_target_trajectory.is_center_point_add) m_target_trajectory.self.markers.push_back(m_target_trajectory.center_point);
    if (m_target_trajectory.is_txt_add) m_target_trajectory.self.markers.push_back(m_target_trajectory.txt);
    m_target_trajectory.is_line_strip_add = m_target_trajectory.is_orientation_add = m_target_trajectory.is_center_point_add = m_target_trajectory.is_txt_add = false;;

    m_target_waypoints.self.markers.clear();
    if (m_target_waypoints.is_line_strip_add) m_target_waypoints.self.markers.push_back(m_target_waypoints.line_strip);
    m_target_waypoints.is_line_strip_add = false;

    m_roi_line.self.markers.clear();
    if (m_roi_line.is_line_strip_add) m_roi_line.self.markers.push_back(m_roi_line.line_strip);
    if (m_roi_line.is_center_point_add) m_roi_line.self.markers.push_back(m_roi_line.center_point);
    if (m_roi_line.is_roi_add) m_roi_line.self.markers.push_back(m_roi_line.roi);
    m_roi_line.is_line_strip_add = m_roi_line.is_center_point_add  = m_roi_line.is_roi_add = false;

    visualization_msgs::MarkerArray visualization_markers;
    visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                        m_target_trajectory.self.markers.begin(), m_target_trajectory.self.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                        m_ego_trajectory.self.markers.begin(), m_ego_trajectory.self.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        m_target_waypoints.self.markers.begin(), m_target_waypoints.self.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        m_roi_line.self.markers.begin(), m_roi_line.self.markers.end());

    m_markers_pub.publish(visualization_markers);
}


void VisualizeControl::EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr)
{
    // center_point
    m_ego_trajectory.center_point.header.frame_id = current_pose_ptr->header.frame_id;
    m_ego_trajectory.center_point.header.stamp = ros::Time(0);

    m_ego_trajectory.center_point.pose = current_pose_ptr->pose;
    m_ego_trajectory.is_center_point_add = true;

    // orientation
    double arrow_size = m_utils.VelNomalize(m_ego_vehicle_speed_kmh);
    m_ego_trajectory.orientation.header.frame_id = current_pose_ptr->header.frame_id;
    m_ego_trajectory.orientation.header.stamp = ros::Time(0);
    
    m_ego_trajectory.orientation.scale.x = arrow_size;
    m_ego_trajectory.orientation.pose = current_pose_ptr->pose;
    m_ego_trajectory.is_orientation_add = true;

    // txt
    m_ego_trajectory.txt.header.frame_id = current_pose_ptr->header.frame_id;
    m_ego_trajectory.txt.header.stamp = ros::Time(0);

    m_ego_trajectory.txt.pose = current_pose_ptr->pose;
    m_ego_trajectory.txt.pose.position.z += 0.8;
    m_ego_trajectory.txt.text = "v_xyz: " + m_utils.ToString(m_ego_vehicle_speed_kmh) + " [kmh]\np_z: " + m_utils.ToString(current_pose_ptr->pose.position.z) + " [m]";
    m_ego_trajectory.is_txt_add = true;
}

void VisualizeControl::EgoVehicleLocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &twist_ptr)
{
    double vel_x = twist_ptr->twist.linear.x;
    double vel_y = twist_ptr->twist.linear.y;
    double vel_z = twist_ptr->twist.linear.z;

    m_ego_vehicle_speed_kmh = m_utils.ms_to_kmh(m_utils.Size(vel_x, vel_y, vel_z));
}

void VisualizeControl::EgoTrajectoryCallback(const uav_msgs::Trajectory::ConstPtr &trajectory_ptr)
{
    if (!trajectory_ptr->is_global){
        // line strip
        m_ego_trajectory.line_strip.header.frame_id = trajectory_ptr->local.header.frame_id;
        m_ego_trajectory.line_strip.header.stamp = ros::Time(0);

        
        m_ego_trajectory.line_strip.points.clear();
        for (auto p : trajectory_ptr->local.poses){
            m_ego_trajectory.line_strip.points.push_back(p.position);
        }
        m_ego_trajectory.is_line_strip_add = true;
    }
}


void VisualizeControl::TargetVehicleLocalNedPositionCallback(const uav_msgs::CarState::ConstPtr &current_point_ptr)
{
    // center_point
    m_target_trajectory.center_point.header.frame_id = current_point_ptr->header.frame_id;
    m_target_trajectory.center_point.header.stamp = ros::Time(0);

    m_target_trajectory.center_point.pose.position.x = current_point_ptr->pose.pose.position.x;
    m_target_trajectory.center_point.pose.position.y = current_point_ptr->pose.pose.position.y;
    m_target_trajectory.center_point.pose.position.z = current_point_ptr->pose.pose.position.z;
    m_target_trajectory.is_center_point_add = true;
}

void VisualizeControl::TargetTrajectoryCallback(const uav_msgs::Trajectory::ConstPtr &trajectory_ptr)
{
    if (!trajectory_ptr->is_global){
        // line strip
        m_target_trajectory.line_strip.header.frame_id = trajectory_ptr->local.header.frame_id;
        m_target_trajectory.line_strip.header.stamp = ros::Time(0);

        m_target_trajectory.line_strip.points.clear();
        for (auto p : trajectory_ptr->local.poses){
            m_target_trajectory.line_strip.points.push_back(p.position);
        }
        m_target_trajectory.is_line_strip_add = true;
    }
}

void VisualizeControl::TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr)
{
    geographic_msgs::GeoPoint geopoint;
    geopoint.latitude = inspva_msg_ptr->latitude;
    geopoint.longitude = inspva_msg_ptr->longitude;
    geopoint.altitude = inspva_msg_ptr->height;
        
    geometry_msgs::PoseStamped posestamped;
    if (!m_utils.IsNan(geopoint)){
        posestamped = m_utils.ConvertToMapFrame(geopoint.latitude, 
                                                geopoint.longitude, 
                                                m_target_height_m_param,
                                                m_home_position);
        
        tf2::Quaternion q;
        q.setRPY(inspva_msg_ptr->roll * M_PI / 180., inspva_msg_ptr->pitch * M_PI / 180., (-1*inspva_msg_ptr->azimuth + 90.0) * M_PI / 180.);
        posestamped.pose.orientation.x = q.x();
        posestamped.pose.orientation.y = q.y();
        posestamped.pose.orientation.z = q.z();
        posestamped.pose.orientation.w = q.w();
    }

    double speed_kmh = m_utils.ms_to_kmh(m_utils.Size(inspva_msg_ptr->east_velocity, inspva_msg_ptr->north_velocity));
    double arrow_size = m_utils.VelNomalize(speed_kmh);

    // orientation
    m_target_trajectory.orientation.header.frame_id = "map";
    m_target_trajectory.orientation.header.stamp = ros::Time(0);

    m_target_trajectory.orientation.scale.x = arrow_size;
    m_target_trajectory.orientation.pose = posestamped.pose;
    m_target_trajectory.is_orientation_add = true;

    // center_point
    m_target_trajectory.center_point.header.frame_id = "map";
    m_target_trajectory.center_point.header.stamp = ros::Time(0);

    m_target_trajectory.center_point.pose = posestamped.pose;
    m_target_trajectory.is_center_point_add = true;

    // txt
    m_target_trajectory.txt.header.frame_id = "map";
    m_target_trajectory.txt.header.stamp = ros::Time(0);

    m_target_trajectory.txt.pose = posestamped.pose;
    m_target_trajectory.txt.pose.position.z += 0.8;
    m_target_trajectory.txt.text = "v_xy: " + m_utils.ToString(speed_kmh) + " [kmh]\np_z: " + m_utils.ToString(posestamped.pose.position.z) + " [m]";
    m_target_trajectory.is_txt_add = true;
}


void VisualizeControl::WaypointsCallback(const uav_msgs::TargetWP::ConstPtr &target_wp_ptr)
{
    // line_strip
    m_target_waypoints.line_strip.header.frame_id = target_wp_ptr->local.header.frame_id;
    m_target_waypoints.line_strip.header.stamp = ros::Time(0);

    m_target_waypoints.line_strip.points.clear();
    for (auto p : target_wp_ptr->local.poses){
        m_target_waypoints.line_strip.points.push_back(p.position);
    }
    m_target_waypoints.is_line_strip_add = true;
}


void VisualizeControl::ROIWaypointsCallback(const uav_msgs::PolyfitLane::ConstPtr &roi_waypoints_ptr)
{
    // line_strip
    m_roi_line.line_strip.header.frame_id = roi_waypoints_ptr->header.frame_id;
    m_roi_line.line_strip.header.stamp = ros::Time(0);

    m_roi_line.line_strip.points.clear();
    for (auto p : roi_waypoints_ptr->points){
        m_roi_line.line_strip.points.push_back(p);
    }
    m_roi_line.is_line_strip_add = true;
}

void VisualizeControl::ROICurrPositionCallback(const nav_msgs::Odometry::ConstPtr &current_point_ptr)
{
    // center_point
    m_roi_line.center_point.header.frame_id = current_point_ptr->header.frame_id;
    m_roi_line.center_point.header.stamp = ros::Time(0);

    m_roi_line.center_point.pose.position.x = current_point_ptr->pose.pose.position.x;
    m_roi_line.center_point.pose.position.y = current_point_ptr->pose.pose.position.y;
    m_roi_line.center_point.pose.position.z = current_point_ptr->pose.pose.position.z;
    m_roi_line.is_center_point_add = true;
}

void VisualizeControl::ROICallback(const uav_msgs::Roi::ConstPtr &roi_ptr)
{
    // roi
    m_roi_line.roi.header.frame_id = roi_ptr->header.frame_id;
    m_roi_line.roi.header.stamp = ros::Time(0);

    m_roi_line.roi.pose.position.x = roi_ptr->pose.position.x;
    m_roi_line.roi.pose.position.y = roi_ptr->pose.position.y;
    m_roi_line.roi.pose.position.z = roi_ptr->pose.position.z;

    m_roi_line.roi.pose.orientation.w = roi_ptr->pose.orientation.w;
    m_roi_line.roi.pose.orientation.x = roi_ptr->pose.orientation.x;
    m_roi_line.roi.pose.orientation.y = roi_ptr->pose.orientation.y;
    m_roi_line.roi.pose.orientation.z = roi_ptr->pose.orientation.z;

    m_roi_line.roi.scale.x = roi_ptr->scale.x;
    m_roi_line.roi.scale.y = roi_ptr->scale.y;
    m_roi_line.roi.scale.z = roi_ptr->scale.z;
    m_roi_line.is_roi_add = true;
}

void VisualizeControl::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[visualize_control] Home set");
    }
}
