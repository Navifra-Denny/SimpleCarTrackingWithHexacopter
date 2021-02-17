#include "control/airsim_offb.h"
#include <math.h>

using namespace msr::airlib;

namespace airsim {
Offboard::Offboard()
{
    GetParam();
    InitClient();
    InitRos();
}

Offboard::~Offboard()
{}

void Offboard::GetParam()
{
    m_nh.getParam("airsim_offb_node/uav_speed", m_speed_ms_param);
    m_nh.getParam("airsim_offb_node/UE_target_name", m_ue_target_name_param);
    m_nh.getParam("airsim_offb_node/uav_name", m_uav_name_param);
    m_nh.getParam("airsim_offb_node/ugv_name", m_target_vehicle_name_param);
    m_nh.getParam("airsim_offb_node/setpoint_pub_interval", m_setpoint_pub_interval_param);
    m_nh.getParam("airsim_offb_node/dt", m_dt_param);
}

void Offboard::InitClient()
{
    try {
        m_client.confirmConnection();
        // auto object_names = m_client.simListSceneObjects();
        // for (auto object_name : object_names){
        //     std::cout << object_name << std::endl;
        // }
    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    m_client.enableApiControl(true); 
    m_client.armDisarm(true);

    std::this_thread::sleep_for(std::chrono::duration<double>(5));
    float takeoffTimeout = 5; 
    m_client.takeoffAsync(takeoffTimeout)->waitOnLastTask();

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1/m_setpoint_pub_interval_param);

    // wait for FCU connection
    while(ros::ok() && (RpcLibClientBase::ConnectionState::Connected != m_client.getConnectionState())){
        ros::spinOnce();
        rate.sleep();
    }

    const float START_HEIGHT_M = -5.0;
    const float START_VELOCITY_MS = 3.0;
    YawMode yaw_mode(true, 0);

    m_local_setpoint.position.x = 0.0;
    m_local_setpoint.position.y = 0.0;
    m_local_setpoint.position.z = START_HEIGHT_M;

    m_client.enableApiControl(true); 
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        m_client.moveToZAsync(START_HEIGHT_M, START_VELOCITY_MS, 5, yaw_mode);
        ros::spinOnce();
        rate.sleep();
    }

    m_client.hoverAsync()->waitOnLastTask();
    m_client.enableApiControl(true); 
}

void Offboard::InitRos()
{
    // package, node, topic name
    std::string car_state_sub_topic_name = "/airsim_node/" + m_target_vehicle_name_param + "/car_state";
	std::string odom_topic_name = "/airsim_node/" + m_uav_name_param + "/odom_local_ned";

    // Initialize subscriber
    m_desired_local_waypoints_pub = m_nh.subscribe<geometry_msgs::PoseArray>
            ("/control/generate_waypoints_node/target_waypoints", 10, boost::bind(&Offboard::DesiredLocalWaypointsCallback, this, _1));
    m_target_vehicle_local_state_sub = m_nh.subscribe<uav_msgs::CarState>
            (car_state_sub_topic_name, 10, boost::bind(&Offboard::TargetVehicleLocalStateCallback, this, _1));
    m_odom_sub = m_nh.subscribe<nav_msgs::Odometry>
            (odom_topic_name, 10, boost::bind(&Offboard::OdomCallback, this, _1));

    // Initialize publisher
    m_target_waypoint_pub = m_nh.createTimer(ros::Duration(m_setpoint_pub_interval_param), &Offboard::TimerCallback, this);
}

void Offboard::DesiredLocalWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr pose_array)
{
    m_local_setpoint = pose_array->poses.back();
}

void Offboard::OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_ptr)
{
    m_ego_vehicle_local_pose = odom_ptr->pose.pose;
}

void Offboard::TargetVehicleLocalStateCallback(const uav_msgs::CarState::ConstPtr &car_state_ptr)
{
    auto position_x = car_state_ptr->pose.pose.position.x;
    auto position_y = car_state_ptr->pose.pose.position.y;
    auto position_z = car_state_ptr->pose.pose.position.z;
    Eigen::Vector3f position = {position_x, position_y, position_z};

    auto orientation_w = car_state_ptr->pose.pose.orientation.w;
    auto orientation_x = car_state_ptr->pose.pose.orientation.x;
    auto orientation_y = car_state_ptr->pose.pose.orientation.y;
    auto orientation_z = car_state_ptr->pose.pose.orientation.z;
    Eigen::Quaternion<float, 2> orientation = {orientation_w, orientation_x, orientation_y, orientation_z};

    msr::airlib::Pose pose(position, orientation);
    auto object_pose = m_client.simGetObjectPose(m_ue_target_name_param);
    pose.position[2] = object_pose.position.z();

    m_client.simSetObjectPose(m_ue_target_name_param, pose, true);
}

void Offboard::TimerCallback(const ros::TimerEvent& event)
{
    // auto euler = m_utils.Quat2Euler(m_local_setpoint.orientation);
    // YawMode yaw_mode(false, euler.y);
    // m_client.moveToPositionAsync((double)m_local_setpoint.position.x, (double)m_local_setpoint.position.y, (double)m_local_setpoint.position.z, 
    //     m_speed_ms_param, Utils::max<float>(), DrivetrainType::ForwardOnly, yaw_mode);

    // ROS_INFO("x: %f, y: %f, z: %f, vel: %f", 
    // (double)m_local_setpoint.position.x, (double)m_local_setpoint.position.y, (double)m_local_setpoint.position.z, (double)m_speed_ms_param);
    
    float dt = m_dt_param;

    auto euler = m_utils.Quat2Euler(m_local_setpoint.orientation);
    YawMode yaw_mode(false, euler.y);
    auto ned_vx = (m_local_setpoint.position.x - m_ego_vehicle_local_pose.position.x)/dt;
    auto ned_vy = (m_local_setpoint.position.y - m_ego_vehicle_local_pose.position.y)/dt;
    auto body_vx = cos(m_utils.Degree2Rad(euler.y))*ned_vx - sin(m_utils.Degree2Rad(euler.y))*ned_vy;
    auto body_vy = sin(m_utils.Degree2Rad(euler.y))*ned_vx + cos(m_utils.Degree2Rad(euler.y))*ned_vy;
    m_client.moveByVelocityZAsync((float)body_vx, (float)body_vy, (float)m_local_setpoint.position.z, dt,
    DrivetrainType::ForwardOnly, yaw_mode);

    ROS_ERROR("dt: %f", (double)dt);
    ROS_INFO("vx: %f, vy: %f, z: %f", 
    (double)body_vx, (double)body_vy, (double)m_local_setpoint.position.z);
}
}