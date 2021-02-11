#include "control/mavros_offb.h"

namespace mavros {
    
Offboard::Offboard() : m_last_request(ros::Time::now())
{
    GetParam();
    InitRos();
    InitClient();
}

Offboard::~Offboard()
{}

void Offboard::GetParam()
{
    m_nh.getParam("mavros_offb_node/uav_speed", m_speed_ms_param);
    m_nh.getParam("mavros_offb_node/UE_target_name", m_ue_target_name_param);
    // m_nh.getParam("mavros_offb_node/uav_name", m_uav_name_param);
    // m_nh.getParam("mavros_offb_node/ugv_name", m_target_vehicle_name_param);
    m_nh.getParam("mavros_offb_node/setpoint_pub_interval", m_setpoint_pub_interval_param);
    m_nh.getParam("mavros_offb_node/dt", m_dt_param);
    m_nh.getParam("mavros_offb_node/init_x", m_init_pos_x_param);
    m_nh.getParam("mavros_offb_node/init_y", m_init_pos_y_param);
    m_nh.getParam("mavros_offb_node/init_z", m_init_pos_z_param);
    ROS_WARN("x: %f", m_init_pos_x_param);
    ROS_WARN("y: %f", m_init_pos_y_param);
    ROS_WARN("z: %f", m_init_pos_z_param);
}

void Offboard::InitRos()
{
    // Initialize subscriber
    m_state_sub = m_nh.subscribe<mavros_msgs::State>("mavros/state", 10, boost::bind(&Offboard::StatusCallback, this, _1));

    // Initialize publisher
    m_local_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    m_arming_serv_client = m_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    m_set_mode_serv_client = m_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    // Time callback
    m_timer = m_nh.createTimer(ros::Duration(m_setpoint_pub_interval_param), &Offboard::TimerCallback, this);
}

void Offboard::InitClient()
{
    ros::Rate rate(1/m_setpoint_pub_interval_param);

    // wait for FCU connection
    while(ros::ok() && !m_current_status.connected){
        ros::spinOnce();
        rate.sleep();
    }

    m_current_pose.pose.position.x = m_init_pos_x_param;
    m_current_pose.pose.position.y = m_init_pos_y_param;
    m_current_pose.pose.position.z = m_init_pos_z_param;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        m_local_pose_pub.publish(m_current_pose);
        ros::spinOnce();
        rate.sleep();
    }
}

void Offboard::StatusCallback(const mavros_msgs::State::ConstPtr &state_ptr)
{
    m_current_status = *state_ptr;
}

void Offboard::TimerCallback(const ros::TimerEvent& event)
{
    if (m_current_status.connected){
        OffboardReConnection();
        m_local_pose_pub.publish(m_current_pose);
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