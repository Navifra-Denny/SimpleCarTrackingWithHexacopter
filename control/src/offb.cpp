#include "control/offb.h"

using namespace msr::airlib;

Offboard::Offboard()
{
    GetParam();
    
    // package, node, topic name
    std::string car_state_sub_topic_name = "/airsim_node/" + m_target_vehicle_name_param + "/car_state";

    // Initialize subscriber
    m_desired_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>
            ("/control/generate_waypoints_node/desired_waypoints", 10, boost::bind(&Offboard::DesiredWaypointsCallback, this, _1));
    m_car_state_sub = m_nh.subscribe<uav_msgs::CarState>
            (car_state_sub_topic_name, 10, boost::bind(&Offboard::CarStateCallback, this, _1));

    
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

    const float START_HEIGHT_M = -5.0;
    const float START_VELOCITY_MS = 3.0;
    YawMode yaw_mode(true, 0);
    m_client.moveToZAsync(START_HEIGHT_M, START_VELOCITY_MS, 5, yaw_mode);

    std::this_thread::sleep_for(std::chrono::duration<double>(2));
    m_client.hoverAsync()->waitOnLastTask();

    m_client.enableApiControl(true); 
}

Offboard::~Offboard()
{}

void Offboard::GetParam()
{
    m_nh.getParam("offb_node/uav_speed", m_speed_ms_param);
    m_nh.getParam("offb_node/UE_target_name", m_ue_target_name_param);
    m_nh.getParam("offb_node/ugv_name", m_target_vehicle_name_param);
}


void Offboard::DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr pose_array)
{
    auto target_position = pose_array->poses.back().position;
    auto target_x = target_position.x;
    auto target_y = target_position.y;
    auto target_z = target_position.z;
    YawMode yaw_mode(true, 0);

    m_client.moveToPositionAsync(target_x, target_y, target_z, m_speed_ms_param, Utils::max<float>(), DrivetrainType::MaxDegreeOfFreedom, yaw_mode,
    -1, 1);
}

void Offboard::CarStateCallback(const uav_msgs::CarState::ConstPtr &car_state_ptr)
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