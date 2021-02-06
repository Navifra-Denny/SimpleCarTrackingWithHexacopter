#include "control/generate_waypoints.h"
#include "uav_msgs/SetLocalPosition.h"
#include "tf2_ros/transform_listener.h"

GenerateWaypoints::GenerateWaypoints()
{
    // Set Param
    SetParam();

    // package, node, topic name
    std::string self_pkg_name = "/control";
    std::string self_node_name = "/generate_waypoints_node";

    // Initialize subscriber
    std::string m_car_state_sub_topic_name = "/airsim_node/" + m_vehicle_name_param + "/car_state";
    m_car_state_sub = m_nh.subscribe<uav_msgs::CarState>(m_car_state_sub_topic_name, 10, boost::bind(&GenerateWaypoints::CarStateCallback, this, _1));

    // Initialize publisher
    m_input_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray>(self_pkg_name + self_node_name + "/input_waypoints", 1);
    m_desired_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray>(self_pkg_name + self_node_name + "/desired_waypoints", 1);
    m_world_enu_input_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray>(self_pkg_name + self_node_name + "/world_enu_input_waypoints", 1);
    m_world_enu_desired_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray>(self_pkg_name + self_node_name + "/world_enu_desired_waypoints", 1);

    // Initialize service client
    m_desired_waypoints_srv_client = m_nh.serviceClient<uav_msgs::SetLocalPosition>("/airsim_node/local_position_goal");
}
GenerateWaypoints::~GenerateWaypoints()
{
}

void GenerateWaypoints::SetParam()
{
    m_nh.getParam("generate_waypoints_node/x_offset_m", m_x_offset_m_param);
    m_nh.getParam("generate_waypoints_node/z_offset_m", m_z_offset_m_param);
    m_nh.getParam("generate_waypoints_node/ugv_name", m_vehicle_name_param);
}

void GenerateWaypoints::CarStateCallback(const uav_msgs::CarState::ConstPtr &car_state_ptr)
{
    geometry_msgs::PoseStamped pose;
    pose.header = car_state_ptr->header;
    pose.pose = car_state_ptr->pose.pose;

    if (!ConvertStateToWaypoints(pose)) ROS_ERROR_STREAM("Fail convert state to pose");
}

bool GenerateWaypoints::ConvertStateToWaypoints(geometry_msgs::PoseStamped pose_stamped)
{
    bool do_push_waypoints = false;
    if (m_input_waypoints.size() < 2)
        do_push_waypoints = true;
    else
    {
        auto distance_m = m_utils.Distance(m_input_waypoints.back().pose.position, pose_stamped.pose.position);
        if (distance_m > 0.2)
            do_push_waypoints = true;
    }

    if (do_push_waypoints)
    {
        m_input_waypoints.push_back(pose_stamped);

        auto desired_point = CreateDesiredWaypoint(pose_stamped);
        m_desired_waypoints.push_back(desired_point);

        uav_msgs::SetLocalPosition desired_local_position;
        desired_local_position.request.x = desired_point.pose.position.x;
        desired_local_position.request.y = desired_point.pose.position.y;
        desired_local_position.request.z = desired_point.pose.position.z;
        auto euler = m_utils.Quat2Euler(desired_point.pose.orientation);
        desired_local_position.request.yaw = euler.y;

        m_desired_waypoints_srv_client.call(desired_local_position);
    }
    else return true;

    geometry_msgs::PoseArray input_pose_array;
    input_pose_array.header = m_input_waypoints.back().header;

    for (auto p : m_input_waypoints) {
        input_pose_array.poses.push_back(p.pose);
    }

    geometry_msgs::PoseArray desired_pose_array;
    desired_pose_array.header = m_desired_waypoints.back().header;

    for (auto p : m_desired_waypoints) {
        desired_pose_array.poses.push_back(p.pose);
    }

    geometry_msgs::PoseArray world_enu_input_pose_array;
    // world_enu_input_pose_array = ConvertWorldEnu(input_pose_array);
    geometry_msgs::PoseArray world_enu_desired_pose_array;
    // world_enu_desired_pose_array = ConvertWorldEnu(desired_pose_array);

    m_input_waypoints_pub.publish(input_pose_array);
    m_desired_waypoints_pub.publish(desired_pose_array);
    // m_world_enu_input_waypoints_pub.publish(world_enu_input_pose_array);
    // m_world_enu_desired_waypoints_pub.publish(world_enu_desired_pose_array);
    m_world_enu_input_waypoints_pub.publish(input_pose_array);
    m_world_enu_desired_waypoints_pub.publish(desired_pose_array);

    return true;
}

geometry_msgs::PoseStamped GenerateWaypoints::CreateDesiredWaypoint(geometry_msgs::PoseStamped pose_stamped)
{
    geometry_msgs::PoseStamped desired_pose_stamped = pose_stamped;
    auto euler = m_utils.Quat2Euler(pose_stamped.pose.orientation);
    desired_pose_stamped.pose.position.x -= m_x_offset_m_param * cos(euler.y);
    desired_pose_stamped.pose.position.y -= m_x_offset_m_param * sin(euler.y);
    desired_pose_stamped.pose.position.z -= m_z_offset_m_param;
    return desired_pose_stamped;
}

geometry_msgs::PoseArray GenerateWaypoints::ConvertWorldEnu(geometry_msgs::PoseArray source_pose_array)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;

    try
    {
        // transformStamped = tfBuffer.lookupTransform("world_ned", source_pose_array.header.frame_id, ros::Time(0));
        transformStamped = tfBuffer.lookupTransform("world_enu", "drone_1", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    ROS_ERROR("x: %f, y: %f, z: %f", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);

    // modify return variable
    geometry_msgs::PoseArray tmp_pose_array;
    return tmp_pose_array;
}