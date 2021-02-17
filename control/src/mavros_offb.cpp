#include "control/mavros_offb.h"
#include "uav_msgs/TargetWPState.h"

namespace mavros {
    
Offboard::Offboard() : 
    m_tfListener(m_tfBuffer),
    m_last_request_time(ros::Time::now()),
    m_last_detected_object_time(ros::Time::now()),
    m_setpoint_pub_interval_param(NAN),
    m_init_gps_lat_param(NAN),
    m_init_gps_lon_param(NAN),
    m_init_gps_alt_param(NAN),
    m_z_offset_param_m(NAN)
{
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    else if (!InitFlag()) ROS_ERROR_STREAM("Fail InitFlag");
    else if (!InitRos()) ROS_ERROR_STREAM("Fail InitRos");
    else if (!InitClient()) ROS_ERROR_STREAM("Fail InitClient");

    m_local_setpoint.header.frame_id = "map";
    m_global_setpoint.header.frame_id = "map";
}

Offboard::~Offboard()
{}

bool Offboard::GetParam()
{
    m_nh.getParam("mavros_offb_node/setpoint_pub_interval", m_setpoint_pub_interval_param);
    m_nh.getParam("mavros_offb_node/init_gps_lat", m_init_gps_lat_param);
    m_nh.getParam("mavros_offb_node/init_gps_lon", m_init_gps_lon_param);
    m_nh.getParam("mavros_offb_node/init_gps_alt", m_init_gps_alt_param);
    m_nh.getParam("mavros_offb_node/z_offset_m", m_z_offset_param_m);
    m_nh.getParam("mavros_offb_node/use_global_setpoint", m_use_global_setpoint_param);
    m_nh.getParam("mavros_offb_node/is_debug_mode", m_is_debug_mode_param);

    if (m_setpoint_pub_interval_param == NAN) { ROS_ERROR_STREAM("m_setpoint_pub_interval_param"); return false; }
    else if (m_init_gps_lat_param == NAN) { ROS_ERROR_STREAM("m_init_gps_lat_param"); return false; }
    else if (m_init_gps_lon_param == NAN) { ROS_ERROR_STREAM("m_init_gps_lon_param"); return false; }
    else if (m_init_gps_alt_param == NAN) { ROS_ERROR_STREAM("m_init_gps_alt_param"); return false; }
    else if (m_z_offset_param_m == NAN) { ROS_ERROR_STREAM("m_z_offset_param_m"); return false; }

    return true;
}

bool Offboard::InitFlag()
{
    m_get_gps_origin = !m_use_global_setpoint_param;
    m_is_hover = true;
    m_is_detected = false;
    m_is_global = false;

    return true;
}

bool Offboard::InitRos()
{
    // Initialize subscriber
    m_state_sub = m_nh.subscribe<mavros_msgs::State>("mavros/state", 10, boost::bind(&Offboard::StatusCallback, this, _1));
    m_target_waypoints_sub = m_nh.subscribe<uav_msgs::TargetWP>
            ("/control/generate_waypoints_node/target_waypoints", 10, boost::bind(&Offboard::TargetWaypointsCallback, this, _1));
            
    m_current_global_pose_sub = m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&Offboard::EgoVehicleGlobalPositionCallback, this, _1));
    m_debugging_sub = m_nh.subscribe<std_msgs::String>("/control/char_pub_node/chatter", 10, boost::bind(&Offboard::DebuggingStringCallback, this, _1));

    // Initialize publisher
    m_local_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    m_global_pose_pub = m_nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    m_gp_origin_pub = m_nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
    m_home_pub = m_nh.advertise<mavros_msgs::HomePosition>("/mavros/home_position/set", 10);

    // Initialize service client
    m_arming_serv_client = m_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    m_set_mode_serv_client = m_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    // Time callback
    m_timer = m_nh.createTimer(ros::Duration(m_setpoint_pub_interval_param), &Offboard::OffboardTimeCallback, this);

    return true;
}

bool Offboard::InitClient()
{
    ros::Rate rate(1/m_setpoint_pub_interval_param);
    if (m_setpoint_pub_interval_param == 0.0) return false;

    // wait for FCU
    while(ros::ok() && !m_current_status.connected){
        ros::spinOnce();
        rate.sleep();
    }

    m_gp_origin.position.latitude = m_init_gps_lat_param;
    m_gp_origin.position.longitude = m_init_gps_lon_param;
    m_gp_origin.position.altitude = m_init_gps_alt_param;
    m_home_pos.geo.latitude = m_init_gps_lat_param;
    m_home_pos.geo.longitude = m_init_gps_lon_param;
    m_home_pos.geo.altitude = m_init_gps_alt_param;
    m_gp_origin_pub.publish(m_gp_origin);
    m_home_pub.publish(m_home_pos);

    return true;
}

void Offboard::OffboardTimeCallback(const ros::TimerEvent& event)
{
    if (m_is_debug_mode_param){
        if (m_debugging_msg == "OFFBOARD"){
            OffboardReConnection();
            m_is_hover = false;
        }
        // Just hovering in all cases, not just MANUAL mode
        else {
            m_is_hover = true;
        }
        PublishSetpoint();
    }
    else {
        if (m_current_status.mode == "OFFBOARD"){
            OffboardReConnection();

            m_is_hover = false;
        }
        // Just hovering in all cases, not just MANUAL mode
        else {
            m_is_hover = true;
        }
        PublishSetpoint();
    }
}

void Offboard::StatusCallback(const mavros_msgs::State::ConstPtr &state_ptr)
{
    m_current_status = *state_ptr;
}

void Offboard::TargetWaypointsCallback(const uav_msgs::TargetWP::ConstPtr &target_wp_ptr)
{
    uav_msgs::TargetWPState target_wp_state = target_wp_ptr->state;

    m_is_global = target_wp_state.is_global;
    if (m_is_global){
        geographic_msgs::GeoPath target_waypoints = target_wp_ptr->global;
    }
    else {
        geometry_msgs::PoseArray waypoints = target_wp_ptr->local;
        geometry_msgs::PoseStamped waypoint;
        waypoint.header = waypoints.header;
        waypoint.pose = waypoints.poses.back();

        if (!m_utils.IsNan(waypoint.pose.position)){

            if (m_local_setpoint.header.frame_id != waypoint.header.frame_id){

                geometry_msgs::TransformStamped transformStamped;
                geometry_msgs::Pose transformed_pose;
                try{
                    transformStamped = m_tfBuffer.lookupTransform(m_local_setpoint.header.frame_id, waypoint.header.frame_id, ros::Time(0));
                    tf2::doTransform(waypoint.pose, transformed_pose, transformStamped);
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(1.0).sleep();
                }
                m_local_setpoint.pose = transformed_pose;
            }
            else{
                m_local_setpoint.pose = waypoint.pose;
            }
        }
    }
}

void Offboard::DesiredGlobalWaypointCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_ptr)
{
    m_last_detected_object_time = ros::Time::now();

    m_global_setpoint.header.frame_id = "map";
    m_global_setpoint.pose.position.latitude = inspva_ptr->latitude;
    m_global_setpoint.pose.position.longitude = inspva_ptr->longitude;
    m_global_setpoint.pose.position.altitude = m_z_offset_param_m;
}

void Offboard::EgoVehicleGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &current_pose_ptr)
{
    m_ego_vehicle.global.pose.position.latitude = current_pose_ptr->latitude;
    m_ego_vehicle.global.pose.position.longitude = current_pose_ptr->longitude;
    m_ego_vehicle.global.pose.position.altitude = current_pose_ptr->altitude;
}

void Offboard::DebuggingStringCallback(const std_msgs::String::ConstPtr &debugging_msgs)
{
    if (debugging_msgs->data == "offboard") m_debugging_msg = "OFFBOARD";
    else if (debugging_msgs->data == "manual") m_debugging_msg = "MANUAL";
}

void Offboard::OffboardReConnection()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if( m_current_status.mode != "OFFBOARD" &&
        (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
        if(m_set_mode_serv_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        m_last_request_time = ros::Time::now();
    } else {
        if(!m_current_status.armed &&
            (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
            if( m_arming_serv_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            m_last_request_time = ros::Time::now();
        }
    }
}

void Offboard::PublishSetpoint()
{
    if (m_is_global) m_global_pose_pub.publish(m_global_setpoint);
    else if (!m_is_global) m_local_pose_pub.publish(m_local_setpoint);
}
}