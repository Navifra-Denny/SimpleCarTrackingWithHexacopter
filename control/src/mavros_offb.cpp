#include "control/mavros_offb.h"

namespace mavros {
    
Offboard::Offboard() : 
    m_tfListener(m_tfBuffer),
    m_last_request(ros::Time::now()),
    m_speed_ms_param(NAN),
    m_uav_name_param("missing"),
    m_target_vehicle_name_param("missing"),
    m_setpoint_pub_interval_param(NAN),
    m_dt_param(NAN),
    m_init_pos_x_param(NAN),
    m_init_pos_y_param(NAN),
    m_init_pos_z_param(NAN)
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
    m_nh.getParam("mavros_offb_node/uav_speed", m_speed_ms_param);
    m_nh.getParam("mavros_offb_node/uav_name", m_uav_name_param);
    m_nh.getParam("mavros_offb_node/ugv_name", m_target_vehicle_name_param);
    m_nh.getParam("mavros_offb_node/setpoint_pub_interval", m_setpoint_pub_interval_param);
    m_nh.getParam("mavros_offb_node/dt", m_dt_param);
    m_nh.getParam("mavros_offb_node/init_x", m_init_pos_x_param);
    m_nh.getParam("mavros_offb_node/init_y", m_init_pos_y_param);
    m_nh.getParam("mavros_offb_node/init_z", m_init_pos_z_param);

    if (m_speed_ms_param == NAN ||
        m_uav_name_param == "missing" ||
        m_target_vehicle_name_param == "missing" ||
        m_setpoint_pub_interval_param == NAN ||
        m_dt_param == NAN ||
        m_init_pos_x_param == NAN ||
        m_init_pos_y_param == NAN ||
        m_init_pos_z_param == NAN)
    {
        return false;
    }

    return true;
}

bool Offboard::InitFlag()
{
    m_is_ready_to_flight = false;

    return true;
}

bool Offboard::InitRos()
{
    // Initialize subscriber
    m_state_sub = m_nh.subscribe<mavros_msgs::State>("mavros/state", 10, boost::bind(&Offboard::StatusCallback, this, _1));
    m_desired_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>
            ("/control/generate_waypoints_node/desired_waypoints", 10, boost::bind(&Offboard::DesiredWaypointsCallback, this, _1));
    m_current_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&Offboard::PositionCallback, this, _1));

    // Initialize publisher
    m_local_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

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

    // wait for FCU connection
    while(ros::ok() && !m_current_status.connected){
        ros::spinOnce();
        rate.sleep();
    }

    m_target_pose.pose.position.x = m_init_pos_x_param;
    m_target_pose.pose.position.y = m_init_pos_y_param;
    m_target_pose.pose.position.z = m_init_pos_z_param;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        m_local_pose_pub.publish(m_target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    m_is_ready_to_flight = true;

    return true;
}

void Offboard::StatusCallback(const mavros_msgs::State::ConstPtr &state_ptr)
{
    m_current_status = *state_ptr;
}

void Offboard::DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array_ptr)
{
    auto pose = pose_array_ptr->poses.back();
    // pose.position.z *= -1;
    // m_target_pose.pose = pose;
    ROS_ERROR_STREAM(pose_array_ptr->header.frame_id);

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

    m_target_pose.header.frame_id = "map";
    m_target_pose.pose = map_pose;
}

void Offboard::PositionCallback(const geometry_msgs::PoseStamped::ConstPtr &current_pose_ptr)
{
    m_current_pose = *current_pose_ptr;
}

void Offboard::TimerCallback(const ros::TimerEvent& event)
{
    if (m_is_ready_to_flight){
        OffboardReConnection();
        m_local_pose_pub.publish(m_target_pose);
    }
}

void Offboard::OffboardReConnection()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if( m_current_status.mode != "OFFBOARD" &&
        (ros::Time::now() - m_last_request > ros::Duration(5.0))){
        if(m_set_mode_serv_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        m_last_request = ros::Time::now();
    } else {
        if(!m_current_status.armed &&
            (ros::Time::now() - m_last_request > ros::Duration(5.0))){
            if( m_arming_serv_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            m_last_request = ros::Time::now();
        }
    }
}
}