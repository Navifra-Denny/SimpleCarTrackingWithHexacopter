#include "control/generate_waypoints.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>

#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoint.h>

#include "uav_msgs/SetLocalPosition.h"
#include "uav_msgs/TargetWaypoints.h"
#include "uav_msgs/VehicleStateArray.h"

#include <math.h>

using namespace control;

GenerateWaypoints::GenerateWaypoints() :
    m_x_offset_m_param(NAN),
    m_z_offset_m_param(NAN),
    m_vehicle_name_param("missing"),
    m_distance_thresh_param(NAN),
    m_target_wp_pub_interval_param(NAN),
    m_detected_dead_band_param(NAN),
    m_alt_offset_m_param(NAN),
    m_target_height_m_param(NAN),
    m_tfListener(m_tfBuffer)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();
    InitTargetVehicle();
}

GenerateWaypoints::~GenerateWaypoints()
{}

bool GenerateWaypoints::InitFlag()
{
    m_is_hover = true;
    m_is_reached = false;
    m_is_global = false;
    m_global_to_local = false;
    m_is_offset_changed = false;
    m_is_heading_changed = false;
    m_is_home_set = false;
    m_detection_tool_lidar_param = false;
    m_detection_tool_gps_param = false;
    m_detection_tool_airsim_param = false;

    return true;
}

bool GenerateWaypoints::GetParam()
{
    m_nh.getParam("generate_waypoints_node/x_offset_m", m_x_offset_m_param);
    m_nh.getParam("generate_waypoints_node/z_offset_m", m_z_offset_m_param);
    m_nh.getParam("generate_waypoints_node/ugv_name", m_vehicle_name_param);
    m_nh.getParam("generate_waypoints_node/distance_threshold", m_distance_thresh_param);
    m_nh.getParam("generate_waypoints_node/target_wp_pub_interval", m_target_wp_pub_interval_param);
    m_nh.getParam("generate_waypoints_node/detected_dead_band", m_detected_dead_band_param);
    m_nh.getParam("generate_waypoints_node/global_to_local", m_global_to_local_param);    
    m_nh.getParam("generate_waypoints_node/alt_offset_m", m_alt_offset_m_param);
    m_nh.getParam("generate_waypoints_node/target_height_m", m_target_height_m_param);
    m_nh.getParam("generate_waypoints_node/detection_tool_lidar", m_detection_tool_lidar_param);
    m_nh.getParam("generate_waypoints_node/detection_tool_gps", m_detection_tool_gps_param);
    m_nh.getParam("generate_waypoints_node/detection_tool_airsim", m_detection_tool_airsim_param);

    if (__isnan(m_x_offset_m_param)) { ROS_ERROR_STREAM("m_x_offset_m_param is NAN"); return false; }
    else if (__isnan(m_z_offset_m_param)) { ROS_ERROR_STREAM("m_z_offset_m_param is NAN"); return false; }
    else if (m_vehicle_name_param == "missing") { ROS_ERROR_STREAM("m_vehicle_name_param is missing"); return false; }
    else if (__isnan(m_distance_thresh_param)) { ROS_ERROR_STREAM("m_distance_thresh_param is NAN"); return false; }
    else if (__isnan(m_target_wp_pub_interval_param)) { ROS_ERROR_STREAM("m_target_wp_pub_interval_param is NAN"); return false; }
    else if (__isnan(m_detected_dead_band_param)) { ROS_ERROR_STREAM("m_detected_dead_band_param is NAN"); return false; }
    else if (__isnan(m_alt_offset_m_param)) { ROS_ERROR_STREAM("m_alt_offset_m_param is NAN"); return false; }
    else if (__isnan(m_target_height_m_param)) { ROS_ERROR_STREAM("m_target_height_m_param is NAN"); return false; }

    m_global_to_local = m_global_to_local_param;
    
    if (m_detection_tool_airsim_param) SELECTED_TOOL = (int)DetectionTool::AirSim;
    else if (m_detection_tool_lidar_param) SELECTED_TOOL = (int)DetectionTool::LiDAR;
    else if (m_detection_tool_gps_param) SELECTED_TOOL = (int)DetectionTool::GPS;

    return true;
}

bool GenerateWaypoints::InitROS()
{
    // package, node, topic name
    std::string node_name_with_namespace = ros::this_node::getName();
    std::string m_car_state_sub_topic_name = "/airsim_node/" + m_vehicle_name_param + "/car_state";

    // Initialize subscriber
    m_airsim_based_target_local_state_sub = 
        m_nh.subscribe<uav_msgs::CarState>(m_car_state_sub_topic_name, 10, boost::bind(&GenerateWaypoints::TargetVehicleLocalStateCallback, this, _1));
    m_lidar_based_target_local_state_sub = 
        m_nh.subscribe<uav_msgs::TargetState>("/tracking/target_state_msg", 10, boost::bind(&GenerateWaypoints::LTargetVehicleLocalStateCallback, this, _1));
    m_gps_based_target_global_position_sub = 
        m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&GenerateWaypoints::TargetVehicleGlobalStateCallback, this, _1));

    m_ego_local_pose_sub = 
        m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&GenerateWaypoints::EgoVehicleLocalPositionCallback, this, _1));
    m_ego_global_pose_sub = 
        m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&GenerateWaypoints::EgoVehicleGlobalPositionCallback, this, _1));        
    m_ego_local_vel_sub = m_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, boost::bind(&GenerateWaypoints::EgoVehicleLocalVelocityCallback, this, _1));
    
    m_chatter_sub = 
        m_nh.subscribe<uav_msgs::Chat>("/control/char_pub_node/chatter", 10, boost::bind(&GenerateWaypoints::ChatterCallback, this, _1));
    m_init_pose_sub = 
        m_nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, boost::bind(&GenerateWaypoints::InitPoseCallback, this, _1));
    m_home_position_sub = 
        m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&GenerateWaypoints::HomePositionCallback, this, _1));
    
    ros::Rate rate(10);
    while (ros::ok() && !m_is_home_set){
        ros::spinOnce();
        rate.sleep();
    }

    // Initialize publisher
    m_target_states_pub = m_nh.advertise<uav_msgs::VehicleStateArray>(node_name_with_namespace + "/target_states", 1);
    m_ego_state_pub = m_nh.advertise<uav_msgs::VehicleState>(node_name_with_namespace + "/ego_state", 1);
    m_target_waypoints_pub = m_nh.advertise<uav_msgs::TargetWaypoints>(node_name_with_namespace + "/target_waypoints", 1);
    m_gen_wp_state_pub = m_nh.advertise<uav_msgs::GenerateWaypointState>(node_name_with_namespace + "/gen_wp_state", 1);

    // Time callback
    m_generate_waypoints_timer = m_nh.createTimer(ros::Duration(m_target_wp_pub_interval_param), &GenerateWaypoints::GenerateWaypointsTimerCallback, this);

    return true;
}

bool GenerateWaypoints::InitTargetVehicle()
{
    m_target_wp.local.header.frame_id = "map";
    m_ego_vehicle.local_trajectory.header.frame_id = "map";

    for (int tool = (int)DetectionTool::AirSim; tool < (int)DetectionTool::ItemNum; tool++){
        control::TargetState target_state;
        target_state.is_detected = false;
        target_state.last_detected_time = ros::Time(0);
        target_state.tool = DetectionTool(tool);
        target_state.local_trajectory.header.frame_id = "map";
        m_target_vehicles.push_back(target_state);
    }

    return true;
}


void GenerateWaypoints::GenerateWaypointsTimerCallback(const ros::TimerEvent& event)
{
    AddPoseToTrajectory(m_ego_vehicle.local_trajectory, m_ego_vehicle.local);

    int tool = (int)DetectionTool::AirSim;
    for (auto &target_vehicle : m_target_vehicles){

        bool is_selected_tool = (tool == SELECTED_TOOL) ? true : false;
        if (is_selected_tool){

            if (IsDetected(target_vehicle, is_selected_tool)){
                AddPoseToTrajectory(target_vehicle.local_trajectory, target_vehicle.local);
                
                if (m_detection_tool_gps_param){
                    if (IsGlobalToLocal()){
                        AddTargetWaypoint(m_target_wp, target_vehicle.local, target_vehicle.velocity);
                    }
                    else{
                        AddTargetWaypoint(m_target_wp, target_vehicle.global, target_vehicle.velocity);
                    }
                }
                else if (m_detection_tool_airsim_param || m_detection_tool_lidar_param){
                    AddTargetWaypoint(m_target_wp, target_vehicle.local, target_vehicle.velocity);
                }
            }
            else{
                AddTargetWaypoint(m_target_wp, m_ego_vehicle.local);
            }

        }
        else{
            if (IsDetected(target_vehicle, is_selected_tool)){
                AddPoseToTrajectory(target_vehicle.local_trajectory, target_vehicle.local);
            }
        }
        tool++;
    }
    
    Publish();
}


void GenerateWaypoints::EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr)
{
    m_ego_vehicle.local = *current_pose_ptr;
    IsReached(m_target_wp, m_ego_vehicle.local.pose.position);
}

void GenerateWaypoints::EgoVehicleGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &current_pose_ptr)
{
    geographic_msgs::GeoPoint geopoint;
    geopoint.latitude = current_pose_ptr->latitude;
    geopoint.longitude = current_pose_ptr->longitude;
    geopoint.altitude = current_pose_ptr->altitude;    

    geometry_msgs::PoseStamped converted_pose;
    if (!m_utils.IsNan(geopoint)){
        converted_pose = m_utils.ConvertToMapFrame(geopoint.latitude, geopoint.longitude, m_target_height_m_param, m_home_position);
    }
    m_complement_x = m_ego_vehicle.local.pose.position.x - converted_pose.pose.position.x;
    m_complement_y = m_ego_vehicle.local.pose.position.y - converted_pose.pose.position.y;
}

void GenerateWaypoints::EgoVehicleLocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &twist_ptr)
{
    m_ego_vehicle.velocity = twist_ptr->twist;
}

void GenerateWaypoints::TargetVehicleLocalStateCallback(const uav_msgs::CarState::ConstPtr &target_state_ptr)
{
    m_target_vehicles[(int)DetectionTool::AirSim].last_detected_time = ros::Time::now();
    if (SELECTED_TOOL == (int)DetectionTool::AirSim){
        m_is_global = false;
    }

    m_target_vehicles[(int)DetectionTool::AirSim].local.header = target_state_ptr->header;
    m_target_vehicles[(int)DetectionTool::AirSim].local.pose = target_state_ptr->pose.pose;
}

void GenerateWaypoints::LTargetVehicleLocalStateCallback(const uav_msgs::TargetState::ConstPtr &target_state_ptr)
{
    if (m_utils.IsValidPos(target_state_ptr->pose.pose)){
        m_target_vehicles[(int)DetectionTool::LiDAR].last_detected_time = ros::Time::now();
        if (SELECTED_TOOL == (int)DetectionTool::LiDAR){
            m_is_global = false;
        }

        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::Pose transformed_pose;
        try{
            transformStamped = m_tfBuffer.lookupTransform("base_link", "map", ros::Time(0));
            tf2::doTransform(target_state_ptr->pose.pose, transformed_pose, transformStamped);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        auto base_link_steering_rad = m_utils.NormalizedSteeringAngleRad(transformed_pose.position.x, transformed_pose.position.y);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, base_link_steering_rad);
        geometry_msgs::Pose base_link_pose;
        base_link_pose.position.x = 0.0;
        base_link_pose.position.y = 0.0;
        base_link_pose.position.z = 0.0;

        base_link_pose.orientation.x = q.x();
        base_link_pose.orientation.y = q.y();
        base_link_pose.orientation.z = q.z();
        base_link_pose.orientation.w = q.w();

        geometry_msgs::Pose enu_pose;
        try{
            transformStamped = m_tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            tf2::doTransform(base_link_pose, enu_pose, transformStamped);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        if (target_state_ptr->pose.header.frame_id.empty()){
            ROS_ERROR_STREAM("LiDAR based target vehicle frame id is missing");
        }
        else{
            m_target_vehicles[(int)DetectionTool::LiDAR].local.header = target_state_ptr->pose.header;
            m_target_vehicles[(int)DetectionTool::LiDAR].local.pose.position = target_state_ptr->pose.pose.position;
            m_target_vehicles[(int)DetectionTool::LiDAR].local.pose.orientation = enu_pose.orientation;
            m_target_vehicles[(int)DetectionTool::LiDAR].velocity = target_state_ptr->velocity;
        }
    }
}

void GenerateWaypoints::TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr)
{
    m_target_vehicles[(int)DetectionTool::GPS].last_detected_time = ros::Time::now();
    if (SELECTED_TOOL == (int)DetectionTool::GPS){
        m_is_global = true;
    }

    m_target_vehicles[(int)DetectionTool::GPS].global.header = inspva_msg_ptr->header;
    m_target_vehicles[(int)DetectionTool::GPS].global.pose.position.latitude = inspva_msg_ptr->latitude;
    m_target_vehicles[(int)DetectionTool::GPS].global.pose.position.longitude = inspva_msg_ptr->longitude;
    m_target_vehicles[(int)DetectionTool::GPS].global.pose.position.altitude = inspva_msg_ptr->height;
    
    if (!m_utils.IsNan(m_target_vehicles[(int)DetectionTool::GPS].global.pose.position)){
    
        m_target_vehicles[(int)DetectionTool::GPS].local = m_utils.ConvertToMapFrame(m_target_vehicles[(int)DetectionTool::GPS].global.pose.position.latitude, 
                                                m_target_vehicles[(int)DetectionTool::GPS].global.pose.position.longitude, 
                                                m_target_height_m_param, 
                                                m_home_position);
        m_target_vehicles[(int)DetectionTool::GPS].local.pose.position.x += m_complement_x;
        m_target_vehicles[(int)DetectionTool::GPS].local.pose.position.y += m_complement_y;

        tf2::Quaternion q;
        q.setRPY(inspva_msg_ptr->roll * M_PI / 180., inspva_msg_ptr->pitch * M_PI / 180., (-1*inspva_msg_ptr->azimuth + 90.0) * M_PI / 180.);
        m_target_vehicles[(int)DetectionTool::GPS].local.pose.orientation.x = q.x();
        m_target_vehicles[(int)DetectionTool::GPS].local.pose.orientation.y = q.y();
        m_target_vehicles[(int)DetectionTool::GPS].local.pose.orientation.z = q.z();
        m_target_vehicles[(int)DetectionTool::GPS].local.pose.orientation.w = q.w();
        m_target_vehicles[(int)DetectionTool::GPS].velocity.linear.x = inspva_msg_ptr->east_velocity;
        m_target_vehicles[(int)DetectionTool::GPS].velocity.linear.y = inspva_msg_ptr->north_velocity;
    }
}

void GenerateWaypoints::ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr)
{
    if (chat_ptr->msg == "local") m_global_to_local = true;
    else if (chat_ptr->msg == "global") m_global_to_local = false;
    else if ((chat_ptr->msg == "local_z") || (chat_ptr->msg == "global_z")){
        m_is_offset_changed = true;

        if (chat_ptr->msg == "local_z"){
            m_z_offset_m_param = (float)chat_ptr->point.z;
        }
        else{
            m_alt_offset_m_param = (float)chat_ptr->geopoint.altitude;
        }
    }
    else if (chat_ptr->msg == "tool"){
        if (chat_ptr->tool == "airsim"){
            m_detection_tool_airsim_param = true;
            m_detection_tool_lidar_param = false;
            m_detection_tool_gps_param = false;
            
            SELECTED_TOOL = (int)DetectionTool::AirSim;
        }
        else if (chat_ptr->tool == "lidar"){
            m_detection_tool_airsim_param = false;
            m_detection_tool_lidar_param = true;
            m_detection_tool_gps_param = false;
         
            SELECTED_TOOL = (int)DetectionTool::LiDAR;
        }
        else if (chat_ptr->tool == "gps"){
            m_detection_tool_airsim_param = false;
            m_detection_tool_lidar_param = false;
            m_detection_tool_gps_param = true;

            SELECTED_TOOL = (int)DetectionTool::GPS;
        }
    }
}

void GenerateWaypoints::InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_ptr)
{
    m_is_heading_changed = true;

    // enu orientation
    m_target_orientation = pose_ptr->pose.pose.orientation;
}


void GenerateWaypoints::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[generate_waypoints] Home set");
    }
}

bool GenerateWaypoints::AddPoseToTrajectory(geometry_msgs::PoseArray &pose_array, geometry_msgs::PoseStamped &curr_pose_stamped)
{
    if (!IsValid(pose_array.poses, curr_pose_stamped.pose.position)){
        return false;
    }
    else{
        geometry_msgs::Pose curr_pose;

        if (pose_array.header.frame_id != curr_pose_stamped.header.frame_id){
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::Pose transformed_pose;
            try{
                transformStamped = m_tfBuffer.lookupTransform(pose_array.header.frame_id, curr_pose_stamped.header.frame_id, ros::Time(0));
                tf2::doTransform(curr_pose_stamped.pose, transformed_pose, transformStamped);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            curr_pose = transformed_pose;
        }
        else{
            curr_pose = curr_pose_stamped.pose;
        }

        pose_array.header.stamp = ros::Time::now();
        pose_array.poses.push_back(curr_pose);

        return true;
    }
}

bool GenerateWaypoints::AddTargetWaypoint(uav_msgs::TargetWaypoints &target_wp, geometry_msgs::PoseStamped &curr_pose_stamped)
{
    geometry_msgs::Twist zero_feedforward_velocity;
    zero_feedforward_velocity.linear.x = 0;
    zero_feedforward_velocity.linear.y = 0;
    zero_feedforward_velocity.linear.z = 0;
    
    return AddTargetWaypoint(target_wp, curr_pose_stamped, zero_feedforward_velocity);
}

bool GenerateWaypoints::AddTargetWaypoint(uav_msgs::TargetWaypoints &target_wp, geometry_msgs::PoseStamped &curr_pose_stamped, geometry_msgs::Twist &target_vel)
{
    geometry_msgs::Pose curr_pose;

    if (target_wp.local.header.frame_id != curr_pose_stamped.header.frame_id){
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::Pose transformed_pose;
        try{
            transformStamped = m_tfBuffer.lookupTransform(target_wp.local.header.frame_id, curr_pose_stamped.header.frame_id, ros::Time(0));
            tf2::doTransform(curr_pose_stamped.pose, transformed_pose, transformStamped);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        curr_pose = transformed_pose;
    }
    else{
        curr_pose = curr_pose_stamped.pose;
    }

    geometry_msgs::Pose target_pose = GenTargetWaypoint(curr_pose, target_vel);
    
    if(IsValid(target_wp.local.poses, target_pose.position)){
        target_wp.local.header.stamp = ros::Time::now();
        target_wp.local.poses.push_back(target_pose);
        
        double coef_vel = 1.5;
        target_vel.linear.x *= coef_vel;
        target_vel.linear.y *= coef_vel;
        target_vel.linear.z *= coef_vel;
        target_wp.velocity = target_vel;
        
        return true;
    }
    return false;
}
  
bool GenerateWaypoints::AddTargetWaypoint(uav_msgs::TargetWaypoints &target_wp, geographic_msgs::GeoPoseStamped &curr_geo_pose_stamped, geometry_msgs::Twist &target_vel)
{
    geographic_msgs::GeoPoseStamped target_geo_pose = GenTargetWaypoint(curr_geo_pose_stamped);
    if (!m_utils.IsNan(target_geo_pose.pose.position)){
        target_wp.global.header.stamp = ros::Time::now();
        target_wp.global.poses.push_back(target_geo_pose);
        
        target_wp.velocity = target_vel;
        return true;
    }
    return false;
}
                   
geometry_msgs::Pose GenerateWaypoints::GenTargetWaypoint(geometry_msgs::Pose &pose, geometry_msgs::Twist &target_vel)
{
    geometry_msgs::Pose target_pose = pose;

    // target_pose.position frame_id is "map"
    if (m_is_hover){
        target_pose.position.z = m_z_offset_m_param;
        target_pose.orientation = m_target_orientation;
    }
    else {
        double speed_kmh = m_utils.ms_to_kmh(m_utils.Size(target_vel.linear.x, target_vel.linear.y));
        double nomalized_speed_ms = m_utils.VelNomalize(speed_kmh);
        double x_offset_m = m_x_offset_m_param * nomalized_speed_ms;

        auto euler = m_utils.Quat2Euler(pose.orientation);
        target_pose.position.x += x_offset_m * cos(euler.y);
        target_pose.position.y += x_offset_m * sin(euler.y);
        target_pose.position.z = m_z_offset_m_param;
        
        m_target_orientation = target_pose.orientation;
    }
    m_is_offset_changed = false;
    m_is_heading_changed = false;

    return target_pose;
}

geographic_msgs::GeoPoseStamped GenerateWaypoints::GenTargetWaypoint(geographic_msgs::GeoPoseStamped &curr_geo_pose_stamped)
{
    geographic_msgs::GeoPoseStamped target_geo_pose_stamped = curr_geo_pose_stamped;
    
    target_geo_pose_stamped.pose.position.altitude = m_alt_offset_m_param;
    // target_geo_pose_stamped.pose.orientation = m_target_orientation;
    m_is_offset_changed = false;
    m_is_heading_changed = false;
    
    return target_geo_pose_stamped;
}


void GenerateWaypoints::Publish()
{
    uav_msgs::VehicleStateArray target_states_msg;
    for (auto target_vehicle : m_target_vehicles){
        uav_msgs::VehicleState target_state_msg;
        if (target_vehicle.local_trajectory.poses.size() != 0){
            target_state_msg.is_global = false;
            target_state_msg.local_posestamped = target_vehicle.local;
            target_state_msg.local_trajectory = target_vehicle.local_trajectory;
            target_state_msg.velocity = target_vehicle.velocity;
        }
        target_states_msg.vehicle_states.push_back(target_state_msg);
    }
    m_target_states_pub.publish(target_states_msg);
    
    uav_msgs::VehicleState ego_state_msg;
    if (m_ego_vehicle.local_trajectory.poses.size() != 0){
        ego_state_msg.is_global = false;
        ego_state_msg.local_posestamped = m_ego_vehicle.local;
        ego_state_msg.local_trajectory = m_ego_vehicle.local_trajectory;
        ego_state_msg.velocity = m_ego_vehicle.velocity;
        m_ego_state_pub.publish(ego_state_msg);
    }

    m_target_wp.is_global = m_is_global;
    if (m_is_global){
        if (IsValid(m_target_wp.global.poses)){
            m_target_waypoints_pub.publish(m_target_wp);
        }
    }
    else{
        if (IsValid(m_target_wp.local.poses)){
            m_target_waypoints_pub.publish(m_target_wp);
        }
    }

    uav_msgs::GenerateWaypointState gen_wp_state_msg;
    gen_wp_state_msg.is_detected = m_target_vehicles.at(SELECTED_TOOL).is_detected;
    gen_wp_state_msg.is_hover = m_is_hover;
    gen_wp_state_msg.is_reached = m_is_reached;
    gen_wp_state_msg.is_global = m_is_global;
    gen_wp_state_msg.global_to_local = m_global_to_local;
    switch(SELECTED_TOOL){
        case (int)DetectionTool::AirSim:
            gen_wp_state_msg.detection_tool = "AirSim";
            break;
        case (int)DetectionTool::LiDAR:
            gen_wp_state_msg.detection_tool = "LiDAR";
            break;
        case (int)DetectionTool::GPS:
            gen_wp_state_msg.detection_tool = "GPS";
            break;
        default:
            break;
    }
    gen_wp_state_msg.curr_local_position = m_ego_vehicle.local.pose.position;
    gen_wp_state_msg.curr_global_position = m_ego_vehicle.global.pose.position;
    if (m_target_wp.local.poses.size() != 0) gen_wp_state_msg.local_waypoint_position = m_target_wp.local.poses.back().position;
    if (m_target_wp.global.poses.size() != 0) gen_wp_state_msg.global_waypoint_position = m_target_wp.global.poses.back().pose.position;
    m_gen_wp_state_pub.publish(gen_wp_state_msg);
}

bool GenerateWaypoints::IsValid(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point curr_position)
{
    if (m_utils.IsNan(curr_position)){
        return false;
    }

    if (poses.size() > 0){
        auto distance_m = m_utils.Distance3D(poses.back().position, curr_position);
        if (distance_m < m_distance_thresh_param){
            return false;
        }
    }
    return true;
}

bool GenerateWaypoints::IsValid(std::vector<geographic_msgs::GeoPoseStamped> &poses, geographic_msgs::GeoPoint curr_position)
{
    if (m_utils.IsNan(curr_position)){
        return false;
    }

    if (poses.size() > 0){
        auto distance_m = m_utils.DistanceFromLatLonInKm(poses.back().pose.position, curr_position);
        if (distance_m < m_distance_thresh_param){
            return false;
        }
    }
    return true;
}

bool GenerateWaypoints::IsValid(std::vector<geometry_msgs::Pose> &poses)
{
    if (poses.size() == 0){
        ROS_ERROR_STREAM("[generate_waypoints] Waypoints is empty");
        return false;
    }
    else 
        return true;
}

bool GenerateWaypoints::IsValid(std::vector<geographic_msgs::GeoPoseStamped> &poses)
{
    if (poses.size() == 0){
        ROS_ERROR_STREAM("[generate_waypoints] Waypoints is empty");
        return false;
    }
    else 
        return true;
}

bool GenerateWaypoints::IsReached(uav_msgs::TargetWaypoints &wp, geometry_msgs::Point curr_position)
{
    if (wp.local.poses.size() == 0){
        return false;
    }
    // reach to target
    // ditance reflects only x and y values
    auto distance_m = m_utils.Distance3D(wp.local.poses.back().position, curr_position);
    if (distance_m < m_distance_thresh_param){
        m_is_reached = true;
        
        return true;
    }
    else{
        m_is_reached = false;
        
        return false;
    }

}

bool GenerateWaypoints::IsReached(uav_msgs::TargetWaypoints &wp, geographic_msgs::GeoPoint curr_position)
{
    if (wp.global.poses.size() == 0){
        return false;
    }

    // reach to target
    auto distance_m = m_utils.DistanceFromLatLonInKm(wp.global.poses.back().pose.position, curr_position);

    if (distance_m < m_distance_thresh_param){
        m_is_reached = true;
        m_is_hover = true;
        m_is_global = false;

        return true;
    }
    else{
        m_is_reached = false;
        
        return false;
    }
}

bool GenerateWaypoints::IsDetected(TargetState& target_state, bool is_selected)
{
    if((ros::Time::now() - target_state.last_detected_time) > ros::Duration(m_detected_dead_band_param)){
        target_state.is_detected = false;

        if (is_selected){
            m_is_hover = true;
            m_is_global = false;
        }

        return false;
    }
    else{
        target_state.is_detected = true;

        if (is_selected){
            m_is_hover = false;
        }
        
        return true;
    }
}

bool GenerateWaypoints::IsGlobalToLocal()
{
    if (m_global_to_local){
        m_is_global = false;

        return true;
    }
    else{
        m_is_global = true;

        return false;
    }
}
