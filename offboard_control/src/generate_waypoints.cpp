#include "generate_waypoints.h"
#include <offboard_control/SetLocalPosition.h>


// #include <math.h>

// std::deque<Vector3r> points;

GenerateWaypoints::GenerateWaypoints()
{
    // Initialize subscriber 
	m_car_state_sub = m_nh.subscribe<offboard_control::CarState>("/airsim_node/drone_1/car_state", 10, boost::bind(&GenerateWaypoints::CarStateCallback, this, _1));

    // Initialize publisher 
    m_input_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray> ("/generate_waypoints_node/input_waypoints", 1);
    m_desired_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray> ("/generate_waypoints_node/desired_waypoints", 1);

    // Initialize service client
    m_desired_waypoints_srv_client = m_nh.serviceClient<offboard_control::SetLocalPosition>("/airsim_node/local_position_goal");

    // Set Param
    SetParam();
}
GenerateWaypoints::~GenerateWaypoints()
{}

void GenerateWaypoints::SetParam()
{
	m_nh.getParam("generate_waypoints_node/x_offset_m", m_x_offset_m_param);
	m_nh.getParam("generate_waypoints_node/z_offset_m", m_z_offset_m_param);
}


void GenerateWaypoints::CarStateCallback(const offboard_control::CarState::ConstPtr &car_state)
{
    auto pose = car_state->pose.pose;
    if (!ConvertStateToWaypoints(pose)) ROS_ERROR_STREAM("Fail convert state to pose");    
}

bool GenerateWaypoints::ConvertStateToWaypoints(geometry_msgs::Pose pose)
{
    bool do_push_waypoints = false;
    if (m_input_waypoints.size() < 2) do_push_waypoints = true;
    else {
        auto distance_m = Distance(m_input_waypoints.back().position, pose.position);
        if (distance_m > 0.2) do_push_waypoints = true;
    }

    if (do_push_waypoints){
        m_input_waypoints.push_back(pose);

        auto desired_point = CreateDesiredWaypoint(pose);
        m_desired_waypoints.push_back(desired_point);
        
        offboard_control::SetLocalPosition desired_local_position;

        desired_local_position.request.x = desired_point.position.x;
        desired_local_position.request.y = desired_point.position.y;
        desired_local_position.request.z = desired_point.position.z;
        desired_local_position.request.yaw = Yaw(desired_point);

        m_desired_waypoints_srv_client.call(desired_local_position);
    }
    else return true;
    
    geometry_msgs::PoseArray input_pose_array;
    for (auto p : m_input_waypoints){
        input_pose_array.poses.push_back(p);
    }
    
    geometry_msgs::PoseArray desired_pose_array;
    for (auto p : m_desired_waypoints){
        desired_pose_array.poses.push_back(p);
    }
    
    m_input_waypoints_pub.publish(input_pose_array);
    m_desired_waypoints_pub.publish(desired_pose_array);

    return true;
}

geometry_msgs::Pose GenerateWaypoints::CreateDesiredWaypoint(geometry_msgs::Pose pose)
{
    geometry_msgs::Pose desired_pose = pose;

    desired_pose.position.x -= m_x_offset_m_param*cos(Yaw(pose));
    desired_pose.position.y -= m_x_offset_m_param*sin(Yaw(pose));
    desired_pose.position.z -= m_z_offset_m_param;
    desired_pose.orientation = pose.orientation;
    return desired_pose;
}

float GenerateWaypoints::Distance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto delta_z = point1.z - point2.z;
    auto distance_m = std::sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
    
    return distance_m;
}

double GenerateWaypoints::Yaw(geometry_msgs::Pose pose)
{
    auto w = pose.orientation.w;
    auto x = pose.orientation.x;
    auto y = pose.orientation.y;
    auto z = pose.orientation.z;

    auto u = 2*(w*z + x*y);
    auto d = 1-2*(y*y + z*z);

    auto yaw = atan2(u, d);
    
    return yaw;
}