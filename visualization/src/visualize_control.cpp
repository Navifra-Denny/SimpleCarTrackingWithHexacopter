#include "visualization/visualize_control.h"

VisualizeControl::VisualizeControl()
{
    SetParam();
    MarkerInit();

    // topic name
    std::string input_curr_position_sub_topic_name = "/airsim_node/" + m_ugv_name_param + "/car_state";
	std::string roi_curr_position_sub_topic_name = "/airsim_node/" + m_uav_name_param + "/odom_local_ned";

    // subscriber init
	m_input_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>("/generate_waypoints_node/input_waypoints", 1, boost::bind(&VisualizeControl::InputWaypointsCallback, this, _1));
	m_input_curr_position_sub = m_nh.subscribe<uav_msgs::CarState>(input_curr_position_sub_topic_name, 10, boost::bind(&VisualizeControl::InputPositionCallback, this, _1));
	m_desired_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>("/generate_waypoints_node/desired_waypoints", 1, boost::bind(&VisualizeControl::DesiredWaypointsCallback, this, _1));
	m_roi_waypoints_sub = m_nh.subscribe<uav_msgs::PolyfitLane>("/extract_lane_node/poly_fit_lane", 1, boost::bind(&VisualizeControl::ROIWaypointsCallback, this, _1));
	m_roi_curr_position_sub = m_nh.subscribe<nav_msgs::Odometry>(roi_curr_position_sub_topic_name, 1, boost::bind(&VisualizeControl::ROICurrPositionCallback, this, _1));
	m_roi_box_sub = m_nh.subscribe<uav_msgs::Roi>("/extract_lane_node/roi", 1, boost::bind(&VisualizeControl::ROICallback, this, _1));

    // publisher init
    m_markers_pub = m_nh.advertise<visualization_msgs::MarkerArray> ("/Marker/control_marekrs", 1);
    m_roi_line_update_timer = m_nh.createTimer(ros::Duration(0.1), &VisualizeControl::TimerCallback, this);
}

VisualizeControl::~VisualizeControl()
{}

void VisualizeControl::SetParam()
{
	m_nh.getParam("visualization_control_node/uav_name", m_ugv_name_param);
	m_nh.getParam("visualization_control_node/ugv_name", m_uav_name_param);
}

void VisualizeControl::MarkerInit()
{
    //// input line
    m_input_line.line_strip.ns = "input_line/lane";
    m_input_line.line_strip.id = 0;
    m_input_line.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_input_line.line_strip.action = visualization_msgs::Marker::ADD;
    m_input_line.line_strip.scale.x = 0.05;
    m_input_line.line_strip.pose.orientation.w = 1.0;
    m_input_line.line_strip.pose.orientation.x = 0.0;
    m_input_line.line_strip.pose.orientation.y = 0.0;
    m_input_line.line_strip.pose.orientation.z = 0.0;
    m_input_line.line_strip.color.r = 0.0f;
    m_input_line.line_strip.color.g = 1.0f;
    m_input_line.line_strip.color.b = 0.0f;
    m_input_line.line_strip.color.a = 1.0;
    m_input_line.line_strip.lifetime = ros::Duration();

    m_input_line.orientation.ns = "input_line/orientation";
    m_input_line.orientation.id = 0;
    m_input_line.orientation.type = visualization_msgs::Marker::ARROW;
    m_input_line.orientation.action = visualization_msgs::Marker::ADD;
    m_input_line.orientation.scale.x = 1.0;
    m_input_line.orientation.scale.y = 0.2;
    m_input_line.orientation.scale.z = 0.2;
    m_input_line.orientation.color.r = 1.0f;
    m_input_line.orientation.color.g = 0.0f;
    m_input_line.orientation.color.b = 0.0f;
    m_input_line.orientation.color.a = 1.0;
    m_input_line.orientation.lifetime = ros::Duration();

    m_input_line.center_point.ns = "input_line/center_point";
    m_input_line.center_point.id = 0;
    m_input_line.center_point.type = visualization_msgs::Marker::SPHERE;
    m_input_line.center_point.action = visualization_msgs::Marker::ADD;
    m_input_line.center_point.scale.x = 0.3;
    m_input_line.center_point.scale.y = 0.3;
    m_input_line.center_point.scale.z = 0.3;
    m_input_line.center_point.pose.orientation.w = 1.0;
    m_input_line.center_point.pose.orientation.x = 0.0;
    m_input_line.center_point.pose.orientation.y = 0.0;
    m_input_line.center_point.pose.orientation.z = 0.0;
    m_input_line.center_point.color.r = 0.0f;
    m_input_line.center_point.color.g = 1.0f;
    m_input_line.center_point.color.b = 0.0f;
    m_input_line.center_point.color.a = 1.0;
    m_input_line.center_point.lifetime = ros::Duration();

    //// desired line
    m_desired_line.line_strip.ns = "desired_line/lane";
    m_desired_line.line_strip.id = 0;
    m_desired_line.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_desired_line.line_strip.action = visualization_msgs::Marker::ADD;
    m_desired_line.line_strip.scale.x = 0.05;
    m_desired_line.line_strip.pose.orientation.w = 1.0;
    m_desired_line.line_strip.pose.orientation.x = 0.0;
    m_desired_line.line_strip.pose.orientation.y = 0.0;
    m_desired_line.line_strip.pose.orientation.z = 0.0;
    m_desired_line.line_strip.color.r = 1.0f;
    m_desired_line.line_strip.color.g = 0.0f;
    m_desired_line.line_strip.color.b = 0.0f;
    m_desired_line.line_strip.color.a = 1.0;
    m_desired_line.line_strip.lifetime = ros::Duration();

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
    m_roi_line.line_strip.color.a = 1.0;
    m_roi_line.line_strip.lifetime = ros::Duration();

    m_roi_line.center_point.ns = "roi_line/center_point";
    m_roi_line.center_point.id = 0;
    m_roi_line.center_point.type = visualization_msgs::Marker::SPHERE;
    m_roi_line.center_point.action = visualization_msgs::Marker::ADD;
    m_roi_line.center_point.scale.x = 0.3;
    m_roi_line.center_point.scale.y = 0.3;
    m_roi_line.center_point.scale.z = 0.3;
    m_roi_line.center_point.pose.orientation.w = 1.0;
    m_roi_line.center_point.pose.orientation.x = 0.0;
    m_roi_line.center_point.pose.orientation.y = 0.0;
    m_roi_line.center_point.pose.orientation.z = 0.0;
    m_roi_line.center_point.color.r = 0.0f;
    m_roi_line.center_point.color.g = 1.0f;
    m_roi_line.center_point.color.b = 1.0f;
    m_roi_line.center_point.color.a = 1.0;
    m_roi_line.center_point.lifetime = ros::Duration();

    m_roi_line.roi.ns = "roi_line/roi";
    m_roi_line.roi.id = 0;
    m_roi_line.roi.type = visualization_msgs::Marker::CUBE;
    m_roi_line.roi.action = visualization_msgs::Marker::ADD;
    m_roi_line.roi.color.r = 1.0f;
    m_roi_line.roi.color.g = 0.0f;
    m_roi_line.roi.color.b = 1.0f;
    m_roi_line.roi.color.a = 0.3;
    m_roi_line.roi.lifetime = ros::Duration();
}

void VisualizeControl::InputWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &input_waypoints_ptr)
{
    // line strip
    m_input_line.line_strip.header.frame_id = input_waypoints_ptr->header.frame_id;
    m_input_line.line_strip.header.stamp = ros::Time(0);

    
    m_input_line.line_strip.points.clear();
    for (auto p : input_waypoints_ptr->poses){
        m_input_line.line_strip.points.push_back(p.position);
    }

    // orientation
    m_input_line.orientation.header.frame_id = input_waypoints_ptr->header.frame_id;
    m_input_line.orientation.header.stamp = ros::Time(0);
    m_input_line.orientation.pose = input_waypoints_ptr->poses.back();
}

void VisualizeControl::InputPositionCallback(const uav_msgs::CarState::ConstPtr &current_point_ptr)
{
    // center_point
    m_input_line.center_point.header.frame_id = current_point_ptr->header.frame_id;
    m_input_line.center_point.header.stamp = ros::Time(0);

    m_input_line.center_point.pose.position.x = current_point_ptr->pose.pose.position.x;
    m_input_line.center_point.pose.position.y = current_point_ptr->pose.pose.position.y;
    m_input_line.center_point.pose.position.z = current_point_ptr->pose.pose.position.z;
}

void VisualizeControl::DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &desired_waypoints_ptr)
{
    // line_strip
    m_desired_line.line_strip.header.frame_id = desired_waypoints_ptr->header.frame_id;
    m_desired_line.line_strip.header.stamp = ros::Time(0);

    m_desired_line.line_strip.points.clear();
    for (auto p : desired_waypoints_ptr->poses){
        m_desired_line.line_strip.points.push_back(p.position);
    }
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
}

void VisualizeControl::ROICurrPositionCallback(const nav_msgs::Odometry::ConstPtr &current_point_ptr)
{
    // center_point
    m_roi_line.center_point.header.frame_id = current_point_ptr->header.frame_id;
    m_roi_line.center_point.header.stamp = ros::Time(0);

    m_roi_line.center_point.pose.position.x = current_point_ptr->pose.pose.position.x;
    m_roi_line.center_point.pose.position.y = current_point_ptr->pose.pose.position.y;
    m_roi_line.center_point.pose.position.z = current_point_ptr->pose.pose.position.z;
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
}

void VisualizeControl::TimerCallback(const ros::TimerEvent& event)
{
    m_input_line.self.markers.clear();
    m_input_line.self.markers.push_back(m_input_line.line_strip);
    m_input_line.self.markers.push_back(m_input_line.orientation);
    m_input_line.self.markers.push_back(m_input_line.center_point);

    m_desired_line.self.markers.clear();
    m_desired_line.self.markers.push_back(m_desired_line.line_strip);

    m_roi_line.self.markers.clear();
    m_roi_line.self.markers.push_back(m_roi_line.line_strip);
    m_roi_line.self.markers.push_back(m_roi_line.center_point);
    m_roi_line.self.markers.push_back(m_roi_line.roi);

    visualization_msgs::MarkerArray visualization_markers;
    visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                        m_input_line.self.markers.begin(), m_input_line.self.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        m_desired_line.self.markers.begin(), m_desired_line.self.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        m_roi_line.self.markers.begin(), m_roi_line.self.markers.end());

    m_markers_pub.publish(visualization_markers);
}