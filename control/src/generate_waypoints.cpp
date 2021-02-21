#include "control/generate_waypoints.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>

#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoint.h>

#include "uav_msgs/SetLocalPosition.h"
#include "uav_msgs/TargetWP.h"
#include "uav_msgs/Trajectory.h"

#include <math.h>

GenerateWaypoints::GenerateWaypoints() : 
    m_last_detected_time(ros::Time(0)),
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

    m_target_wp.local.header.frame_id = "map";
    m_ego_vehicle.local_trajectory.header.frame_id = "map";
    m_target_vehicle.local_trajectory.header.frame_id = "map";
}

GenerateWaypoints::~GenerateWaypoints()
{}

bool GenerateWaypoints::InitFlag()
{
    m_target_wp.state.is_detected = false;
    m_target_wp.state.is_global = false;
    m_target_wp.state.is_hover = true;
    m_is_offset_changed = false;
    m_is_home_set = false;
    m_is_golbal_to_local = false;

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

    if (__isnan(m_x_offset_m_param)) { ROS_ERROR_STREAM("m_x_offset_m_param is NAN"); return false; }
    else if (__isnan(m_z_offset_m_param)) { ROS_ERROR_STREAM("m_z_offset_m_param is NAN"); return false; }
    else if (m_vehicle_name_param == "missing") { ROS_ERROR_STREAM("m_vehicle_name_param is missing"); return false; }
    else if (__isnan(m_distance_thresh_param)) { ROS_ERROR_STREAM("m_distance_thresh_param is NAN"); return false; }
    else if (__isnan(m_target_wp_pub_interval_param)) { ROS_ERROR_STREAM("m_target_wp_pub_interval_param is NAN"); return false; }
    else if (__isnan(m_detected_dead_band_param)) { ROS_ERROR_STREAM("m_detected_dead_band_param is NAN"); return false; }
    else if (__isnan(m_alt_offset_m_param)) { ROS_ERROR_STREAM("m_alt_offset_m_param is NAN"); return false; }
    else if (__isnan(m_target_height_m_param)) { ROS_ERROR_STREAM("m_target_height_m_param is NAN"); return false; }

    m_z_offset_m = m_z_offset_m_param;
    m_x_offset_m = m_x_offset_m_param;
    m_alt_offset_m = m_alt_offset_m_param;
    m_is_golbal_to_local = m_global_to_local_param;

    return true;
}

bool GenerateWaypoints::InitROS()
{
    // package, node, topic name
    std::string self_pkg_name = "/control";
    std::string self_node_name = "/generate_waypoints_node";
    std::string m_car_state_sub_topic_name = "/airsim_node/" + m_vehicle_name_param + "/car_state";

    // Initialize subscriber
    m_target_vehicle_local_state_sub = 
        m_nh.subscribe<uav_msgs::CarState>(m_car_state_sub_topic_name, 10, boost::bind(&GenerateWaypoints::TargetVehicleLocalStateCallback, this, _1));
    m_target_vehicle_global_position_sub = 
        m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&GenerateWaypoints::TargetVehicleGlobalStateCallback, this, _1));
    m_current_local_pose_sub = 
        m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&GenerateWaypoints::EgoVehicleLocalPositionCallback, this, _1));
    m_offset_sub = 
        m_nh.subscribe<uav_msgs::Offset>("/control/char_pub_node/offset", 10, boost::bind(&GenerateWaypoints::OffsetCallback, this, _1));
    m_chatter_sub = 
        m_nh.subscribe<std_msgs::String>("/control/char_pub_node/chatter", 10, boost::bind(&GenerateWaypoints::ChatterCallback, this, _1));
    m_home_position_sub = 
        m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&GenerateWaypoints::HomePositionCallback, this, _1));
    
    ros::Rate rate(10);
    while (ros::ok() && !m_is_home_set){
        ros::spinOnce();
        rate.sleep();
    }

    // Initialize publisher
    m_target_trajectory_pub = m_nh.advertise<uav_msgs::Trajectory>(self_pkg_name + self_node_name + "/target_trajectory", 1);
    m_ego_trajectory_pub = m_nh.advertise<uav_msgs::Trajectory>(self_pkg_name + self_node_name + "/ego_trajectory", 1);
    m_target_waypoints_pub = m_nh.advertise<uav_msgs::TargetWP>(self_pkg_name + self_node_name + "/target_waypoints", 1);

    // Time callback
    m_generate_waypoints_timer = m_nh.createTimer(ros::Duration(m_target_wp_pub_interval_param), &GenerateWaypoints::GenerateWaypointsTimerCallback, this);

    return true;
}

bool GenerateWaypoints::InitTargetVehicle()
{
    return true;
}


void GenerateWaypoints::GenerateWaypointsTimerCallback(const ros::TimerEvent& event)
{
    if((ros::Time::now() - m_last_detected_time) > ros::Duration(m_detected_dead_band_param)){
        m_target_wp.state.is_detected = false;
        m_target_wp.state.is_hover = true;
        m_target_wp.state.is_global = false;

        if (IsValid(m_ego_vehicle.local_trajectory.poses, m_ego_vehicle.local.pose.position)){
            // Adding ego vehicle position point to ego vehicle trajectory when target waypoint is added
            if (AddTargetWaypoint(m_target_wp, m_ego_vehicle.local)){
                AddPointToTrajectory(m_ego_vehicle.local_trajectory, m_ego_vehicle.local);
            }
        }
    }
    else{
        m_target_wp.state.is_detected = true;
        m_target_wp.state.is_hover = false;
        
        if(m_target_wp.state.is_global){
            if (m_is_golbal_to_local){
                m_target_wp.state.is_global = false;
                if (IsValid(m_target_vehicle.local_trajectory.poses, m_target_vehicle.local.pose.position)){
                    if (AddTargetWaypoint(m_target_wp, m_target_vehicle.local, m_target_vehicle.velocity)){
                        AddPointToTrajectory(m_ego_vehicle.local_trajectory, m_ego_vehicle.local);
                        AddPointToTrajectory(m_target_vehicle.local_trajectory, m_target_vehicle.local);
                    }
                }
            }
            else{
                if (IsValid(m_target_vehicle.global_trajectory.poses, m_target_vehicle.global.pose.position)){
                    if (AddTargetWaypoint(m_target_wp, m_target_vehicle.global, m_target_vehicle.velocity)){
                        AddPointToTrajectory(m_ego_vehicle.local_trajectory, m_ego_vehicle.local);
                        AddPointToTrajectory(m_target_vehicle.local_trajectory, m_target_vehicle.local);
                    }
                }
            }
        }
        else{
            if (IsValid(m_target_vehicle.local_trajectory.poses, m_target_vehicle.local.pose.position)){
                // Adding ego vehicle position point to ego vehicle trajectory when target waypoint is added
                if (AddTargetWaypoint(m_target_wp, m_target_vehicle.local, m_target_vehicle.velocity)){
                    AddPointToTrajectory(m_ego_vehicle.local_trajectory, m_ego_vehicle.local);
                    AddPointToTrajectory(m_target_vehicle.local_trajectory, m_target_vehicle.local);
                }
            }
        }
    }

    if (m_target_wp.state.is_global){
        if (IsValid(m_target_wp.global.poses)){
            Publish();
        }
    }
    else{
        if (IsValid(m_target_wp.local.poses)){
            Publish();
        }
    }
}


void GenerateWaypoints::EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr)
{
    m_ego_vehicle.local = *current_pose_ptr;
}

void GenerateWaypoints::TargetVehicleLocalStateCallback(const uav_msgs::CarState::ConstPtr &car_state_ptr)
{
    m_last_detected_time = ros::Time::now();
    m_target_wp.state.is_global = false;

    m_target_vehicle.local.header = car_state_ptr->header;
    m_target_vehicle.local.pose = car_state_ptr->pose.pose;
}

void GenerateWaypoints::TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr)
{
    m_last_detected_time = ros::Time::now();
    m_target_wp.state.is_global = true;

    m_target_vehicle.global.header = inspva_msg_ptr->header;
    m_target_vehicle.global.pose.position.latitude = inspva_msg_ptr->latitude;
    m_target_vehicle.global.pose.position.longitude = inspva_msg_ptr->longitude;
    m_target_vehicle.global.pose.position.altitude = inspva_msg_ptr->height;
    
    if (!m_utils.IsNan(m_target_vehicle.global.pose.position)){
    
        m_target_vehicle.local = m_utils.ConvertToMapFrame(m_target_vehicle.global.pose.position.latitude, 
                                                m_target_vehicle.global.pose.position.longitude, 
                                                m_target_height_m_param, 
                                                m_home_position);
        
        tf2::Quaternion q;
        q.setRPY(inspva_msg_ptr->roll * M_PI / 180., inspva_msg_ptr->pitch * M_PI / 180., (-1*inspva_msg_ptr->azimuth + 90.0) * M_PI / 180.);
        m_target_vehicle.local.pose.orientation.x = q.x();
        m_target_vehicle.local.pose.orientation.y = q.y();
        m_target_vehicle.local.pose.orientation.z = q.z();
        m_target_vehicle.local.pose.orientation.w = q.w();
    }
    
    m_target_vehicle.velocity.linear.x = inspva_msg_ptr->east_velocity;
    m_target_vehicle.velocity.linear.y = inspva_msg_ptr->north_velocity;
}

void GenerateWaypoints::ChatterCallback(const std_msgs::String::ConstPtr &string_ptr)
{
    if (string_ptr->data == "local") m_is_golbal_to_local = true;
    else if (string_ptr->data == "global") m_is_golbal_to_local = false;;
}


void GenerateWaypoints::OffsetCallback(const uav_msgs::Offset::ConstPtr &offset_ptr)
{
    m_is_offset_changed = true;
    
    if (offset_ptr->is_global){
        m_alt_offset_m = (float)offset_ptr->geopoint.altitude;
    }
    else {
        m_z_offset_m = (float)offset_ptr->point.z;
    }
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


bool GenerateWaypoints::AddPointToTrajectory(geometry_msgs::PoseArray &pose_array, geometry_msgs::PoseStamped &curr_pose_stamped)
{
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
}

bool GenerateWaypoints::AddTargetWaypoint(uav_msgs::TargetWP &target_wp, geometry_msgs::PoseStamped &curr_pose_stamped)
{
    geometry_msgs::Twist zero_feedforward_velocity;
    zero_feedforward_velocity.linear.x = 0;
    zero_feedforward_velocity.linear.y = 0;
    zero_feedforward_velocity.linear.z = 0;
    
    return AddTargetWaypoint(target_wp, curr_pose_stamped, zero_feedforward_velocity);
}

bool GenerateWaypoints::AddTargetWaypoint(uav_msgs::TargetWP &target_wp, geometry_msgs::PoseStamped &curr_pose_stamped, geometry_msgs::Twist &target_vel)
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

    geometry_msgs::Pose target_pose = GenTargetWaypoint(curr_pose);
    if (!m_utils.IsNan(target_pose.position)){
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
  
bool GenerateWaypoints::AddTargetWaypoint(uav_msgs::TargetWP &target_wp, geographic_msgs::GeoPoseStamped &curr_geo_pose_stamped, geometry_msgs::Twist &target_vel)
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

geometry_msgs::Pose GenerateWaypoints::GenTargetWaypoint(geometry_msgs::Pose &pose)
{
    geometry_msgs::Pose target_pose = pose;
    auto euler = m_utils.Quat2Euler(pose.orientation);

    // target_pose.position frame_id is "map"
    if (m_target_wp.state.is_hover){
        target_pose.position.z = m_z_offset_m;
    }
    else {
        double speed_kmh = m_utils.ms_to_kmh(m_utils.Size(m_target_vehicle.velocity.linear.x, m_target_vehicle.velocity.linear.y));
        double nomalized_speed_ms = m_utils.VelNomalize(speed_kmh);
        m_x_offset_m = m_x_offset_m_param * nomalized_speed_ms;

        target_pose.position.x += m_x_offset_m * cos(euler.y);
        target_pose.position.y += m_x_offset_m * sin(euler.y);
        target_pose.position.z = m_z_offset_m;
    }
    m_is_offset_changed = false;

    return target_pose;
}

geographic_msgs::GeoPoseStamped GenerateWaypoints::GenTargetWaypoint(geographic_msgs::GeoPoseStamped &curr_geo_pose_stamped)
{
    geographic_msgs::GeoPoseStamped target_geo_pose_stamped = curr_geo_pose_stamped;
    
    target_geo_pose_stamped.pose.position.altitude = m_alt_offset_m;
    m_is_offset_changed = false;

    return target_geo_pose_stamped;
}


void GenerateWaypoints::Publish()
{
    uav_msgs::Trajectory target_traj_msg;
    if (m_target_vehicle.local_trajectory.poses.size() != 0){
        target_traj_msg.is_global = false;
        target_traj_msg.local = m_target_vehicle.local_trajectory;
        m_target_trajectory_pub.publish(target_traj_msg);
    }
    
    uav_msgs::Trajectory ego_traj_msg;
    if (m_ego_vehicle.local_trajectory.poses.size() != 0){
        ego_traj_msg.is_global = false;
        ego_traj_msg.local = m_ego_vehicle.local_trajectory;
        m_ego_trajectory_pub.publish(ego_traj_msg);
    }

    m_target_waypoints_pub.publish(m_target_wp);
}

bool GenerateWaypoints::IsValid(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point curr_position)
{
    if (m_utils.IsNan(curr_position)){
        return false;
    }
    else if (m_is_offset_changed){
        return true;
    }
    else if (poses.size() < 1){
        return true;
    }
    else{
        // ditance reflects only x and y values
        auto distance_m = m_utils.Distance(poses.back().position, curr_position);
        if (distance_m > m_distance_thresh_param){
            return true;
        }
    }
    return false;
}

bool GenerateWaypoints::IsValid(std::vector<geographic_msgs::GeoPoseStamped> &poses, geographic_msgs::GeoPoint curr_position)
{
    if (m_utils.IsNan(curr_position)){
        return false;
    }
    else if (m_is_offset_changed){
        return true;
    }
    else if (poses.size() < 1){
        return true;
    }
    else{
        // ToDo change
        auto distance_m = m_utils.DistanceFromLatLonInKm(poses.back().pose.position, curr_position);
        if (distance_m > m_distance_thresh_param){
            return true;
        }
    }
    return false;
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
