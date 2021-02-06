#include "control/offb.h"

using namespace msr::airlib;

Offboard::Offboard()
{

    // Initialize subscriber
    m_desired_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>
            ("/control/generate_waypoints_node/desired_waypoints", 10, boost::bind(&Offboard::DesiredWaypointsCallback, this, _1));

    
    try {
        m_client.confirmConnection();

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

    std::this_thread::sleep_for(std::chrono::duration<double>(5));
    m_client.hoverAsync()->waitOnLastTask();

    m_client.enableApiControl(true); 
}

Offboard::~Offboard()
{}

void Offboard::SetParam()
{}


void Offboard::DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr pose_array)
{
    auto target_position = pose_array->poses.back().position;
    auto target_x = target_position.x;
    auto target_y = target_position.y;
    auto target_z = target_position.z;

    const float speed = 7.0f;
    m_client.moveToPositionAsync(target_x, target_y, target_z, speed);
}