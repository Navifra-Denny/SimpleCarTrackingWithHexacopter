#include "control/mavros_offb.h"

namespace mavros {
    
Offboard::Offboard() : 
    m_tfListener(m_tfBuffer),
    m_last_request_time(ros::Time::now()),
    m_last_detected_object_time(ros::Time::now()),
    m_setpoint_pub_interval_param(NAN),
    m_init_pos_x_param(NAN),
    m_init_pos_y_param(NAN),
    m_init_pos_z_param(NAN),
    m_init_gps_lat_param(NAN),
    m_init_gps_lon_param(NAN),
    m_init_gps_alt_param(NAN),
    m_z_offset_param_m(NAN)
{
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    else if (!InitFlag()) ROS_ERROR_STREAM("Fail InitFlag");
    else if (!InitRos()) ROS_ERROR_STREAM("Fail InitRos");
    else if (!InitClient()) ROS_ERROR_STREAM("Fail InitClient");
}

Offboard::~Offboard()
{}

bool Offboard::GetParam()
{
    m_nh.getParam("mavros_offb_node/setpoint_pub_interval", m_setpoint_pub_interval_param);
    m_nh.getParam("mavros_offb_node/init_local_x", m_init_pos_x_param);
    m_nh.getParam("mavros_offb_node/init_local_y", m_init_pos_y_param);
    m_nh.getParam("mavros_offb_node/init_local_z", m_init_pos_z_param);
    m_nh.getParam("mavros_offb_node/init_gps_lat", m_init_gps_lat_param);
    m_nh.getParam("mavros_offb_node/init_gps_lon", m_init_gps_lon_param);
    m_nh.getParam("mavros_offb_node/init_gps_alt", m_init_gps_alt_param);
    m_nh.getParam("mavros_offb_node/z_offset_m", m_z_offset_param_m);
    m_nh.getParam("mavros_offb_node/use_global_setpoint", m_use_global_setpoint_param);
    m_nh.getParam("mavros_offb_node/is_debug_mode", m_is_debug_mode_param);

    if (m_setpoint_pub_interval_param == NAN ||
        m_init_pos_x_param == NAN ||
        m_init_pos_y_param == NAN ||
        m_init_pos_z_param == NAN ||
        m_init_gps_lat_param == NAN ||
        m_init_gps_lon_param == NAN ||
        m_init_gps_alt_param == NAN ||
        m_z_offset_param_m == NAN)
    {
        return false;
    }
    ParamLog();

    return true;
}

bool Offboard::InitFlag()
{
    m_get_gps_origin = !m_use_global_setpoint_param;
    m_detected_object = false;
    m_do_takeoff = true;

    return true;
}

bool Offboard::InitRos()
{
    // Initialize subscriber
    m_state_sub = m_nh.subscribe<mavros_msgs::State>("mavros/state", 10, boost::bind(&Offboard::StatusCallback, this, _1));
    m_desired_local_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>
            ("/control/generate_waypoints_node/desired_local_waypoints", 10, boost::bind(&Offboard::DesiredLocalWaypointsCallback, this, _1));
    m_desired_global_waypoint_sub = m_nh.subscribe<novatel_oem7_msgs::INSPVA>
            ("/novatel/oem7/inspva", 10, boost::bind(&Offboard::DesiredGlobalWaypointCallback, this, _1));
    m_current_local_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&Offboard::EgoVehicleLocalPositionCallback, this, _1));
    m_current_global_pose_sub = m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&Offboard::EgoVehicleGlobalPositionCallback, this, _1));
    m_debugging_sub = m_nh.subscribe<std_msgs::String>("/chatter", 10, boost::bind(&Offboard::DebuggingStringCallback, this, _1));
    m_z_target_sub = m_nh.subscribe<geometry_msgs::Point>("/z_target", 10, boost::bind(&Offboard::ZOffsetCallback, this, _1));

    // Initialize publisher
    m_curr_status_pub = m_nh.advertise<uav_msgs::uav_status>("/control/mavros_offb_node/current_status", 10);
    m_local_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    m_global_pose_pub = m_nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    m_gp_origin_pub = m_nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
    m_home_pub = m_nh.advertise<mavros_msgs::HomePosition>("/mavros/home_position/set", 10);

    // Initialize service client
    m_arming_serv_client = m_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    m_set_mode_serv_client = m_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    // Time callback
    m_timer = m_nh.createTimer(ros::Duration(m_setpoint_pub_interval_param), &Offboard::TimerCallback, this);

    return true;
}

bool Offboard::InitClient()
{
    ros::Rate rate(1/m_setpoint_pub_interval_param);
    if (m_setpoint_pub_interval_param == 0.0) return false;

    // wait for FCU and GPS connection
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

    m_local_setpoint.pose.position.x = m_init_pos_x_param;
    m_local_setpoint.pose.position.y = m_init_pos_y_param;
    m_local_setpoint.pose.position.z = m_init_pos_z_param;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        m_local_pose_pub.publish(m_local_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    return true;
}

void Offboard::StatusCallback(const mavros_msgs::State::ConstPtr &state_ptr)
{
    m_current_status = *state_ptr;
}

void Offboard::DesiredLocalWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array_ptr)
{
    m_detected_object = true;
    m_last_detected_object_time = ros::Time::now();

    auto pose = pose_array_ptr->poses.back();

	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::Pose map_pose;
	try{
		transformStamped = m_tfBuffer.lookupTransform("map", pose_array_ptr->header.frame_id, ros::Time(0));
		tf2::doTransform(pose, map_pose, transformStamped);
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

    m_local_setpoint.header.frame_id = "map";
    m_local_setpoint.pose = map_pose;

}

void Offboard::DesiredGlobalWaypointCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_ptr)
{
    m_detected_object = true;
    m_last_detected_object_time = ros::Time::now();

    m_global_setpoint.header.frame_id = "map";
    m_global_setpoint.pose.position.latitude = inspva_ptr->latitude;
    m_global_setpoint.pose.position.longitude = inspva_ptr->longitude;
    m_global_setpoint.pose.position.altitude = m_z_offset_param_m;
}

void Offboard::EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr)
{
    m_ego_vehicle.local_pose = *current_pose_ptr;
}

void Offboard::EgoVehicleGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr &current_pose_ptr)
{
    m_ego_vehicle.global_pose_raw.pose.position.latitude = current_pose_ptr->latitude;
    m_ego_vehicle.global_pose_raw.pose.position.longitude = current_pose_ptr->longitude;
    m_ego_vehicle.global_pose_raw.pose.position.altitude = current_pose_ptr->altitude;
}

void Offboard::DebuggingStringCallback(const std_msgs::String::ConstPtr &debugging_msgs)
{
    if (debugging_msgs->data == "offboard") m_debugging_msg = "OFFBOARD";
    else if (debugging_msgs->data == "manual") m_debugging_msg = "MANUAL";
}

void Offboard::ZOffsetCallback(const geometry_msgs::Point::ConstPtr &z_target_point_ptr)
{
    m_z_offset_param_m = (float)z_target_point_ptr->z;
}

void Offboard::TimerCallback(const ros::TimerEvent& event)
{
    bool do_hover = false;

    if (m_is_debug_mode_param){
        if (m_debugging_msg == "OFFBOARD"){
            OffboardReConnection();
            CheckObjectDetected();
            if (m_detected_object) do_hover = false;
            else do_hover = true;
        }
        else if (m_debugging_msg == "MANUAL"){
            m_do_takeoff = true;
            do_hover = true;
        }
    }
    else {
        if (m_current_status.mode == "OFFBOARD"){
            OffboardReConnection();
            CheckObjectDetected();
            if (m_detected_object) do_hover = false;
            else do_hover = true;
        }
        else if (m_current_status.mode == "MANUAL"){
            m_do_takeoff = true;
            do_hover = true;
        }
    }
    PublishSetpoint(do_hover);
    StatusLog();
    PublishCurrentStatus();
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

void Offboard::CheckObjectDetected()
{
    if ((ros::Time::now() - m_last_detected_object_time) > ros::Duration(5.0)){
        m_detected_object = false;
    }
}

void Offboard::PublishSetpoint(bool do_hover)
{
    if (do_hover){
        geometry_msgs::PoseStamped hover_local_setpoint;
        hover_local_setpoint.pose.position.x = m_ego_vehicle.local_pose.pose.position.x;
        hover_local_setpoint.pose.position.y = m_ego_vehicle.local_pose.pose.position.y;
        hover_local_setpoint.pose.position.z = m_init_pos_z_param;

        m_local_pose_pub.publish(hover_local_setpoint);
    }
    else{
        if (m_use_global_setpoint_param) m_global_pose_pub.publish(m_global_setpoint);
        else if (!m_use_global_setpoint_param) m_local_pose_pub.publish(m_local_setpoint);
    }
}

void Offboard::PublishCurrentStatus()
{
    m_curr_status_pub.publish(m_uav_status_msg);
}

void Offboard::ParamLog()
{
    m_uav_status_msg.is_debug_mode = m_is_debug_mode_param;
    m_uav_status_msg.use_global_setpoint = m_use_global_setpoint_param;
    m_uav_status_msg.init_local_x = m_init_pos_x_param;
    m_uav_status_msg.init_local_y = m_init_pos_y_param;
    m_uav_status_msg.init_local_z = m_init_pos_z_param;
    m_uav_status_msg.init_gps_lat = m_init_gps_lat_param;
    m_uav_status_msg.init_gps_lon = m_init_gps_lon_param;
    m_uav_status_msg.init_gps_alt = m_init_gps_alt_param;

    m_uav_status_msg.setpoint_pub_interval = m_setpoint_pub_interval_param;
}

void Offboard::StatusLog()
{
    m_uav_status_msg.mode = m_current_status.mode;

    m_uav_status_msg.x = m_ego_vehicle.local_pose.pose.position.x;
    m_uav_status_msg.y = m_ego_vehicle.local_pose.pose.position.y;
    m_uav_status_msg.z = m_ego_vehicle.local_pose.pose.position.z;

    m_uav_status_msg.lat = m_ego_vehicle.global_pose_raw.pose.position.latitude;
    m_uav_status_msg.lon = m_ego_vehicle.global_pose_raw.pose.position.longitude;
    m_uav_status_msg.alt = m_ego_vehicle.global_pose_raw.pose.position.altitude;

    m_uav_status_msg.z_offset_m = m_z_offset_param_m;
}
}