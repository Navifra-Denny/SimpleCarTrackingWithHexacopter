#include "control/generate_waypoints.h"
#include "uav_msgs/SetLocalPosition.h"
#include "tf2_ros/transform_listener.h"

GenerateWaypoints::GenerateWaypoints()
{
    GetParam();
    RosInit();
    InitTargetVehicle();
}
GenerateWaypoints::~GenerateWaypoints()
{
}

void GenerateWaypoints::GetParam()
{
    m_nh.getParam("generate_waypoints_node/x_offset_m", m_x_offset_m_param);
    m_nh.getParam("generate_waypoints_node/z_offset_m", m_z_offset_m_param);
    m_nh.getParam("generate_waypoints_node/ugv_name", m_vehicle_name_param);
    m_nh.getParam("generate_waypoints_node/distance_threshold", m_distance_thresh_param);
    m_nh.getParam("generate_waypoints_node/init_gps_lat", m_init_gps_lat_param);
    m_nh.getParam("generate_waypoints_node/init_gps_lon", m_init_gps_lon_param);
    m_nh.getParam("generate_waypoints_node/init_gps_alt", m_init_gps_alt_param);
}

void GenerateWaypoints::RosInit()
{
    // package, node, topic name
    std::string self_pkg_name = "/control";
    std::string self_node_name = "/generate_waypoints_node";

    // Initialize subscriber
    std::string m_car_state_sub_topic_name = "/airsim_node/" + m_vehicle_name_param + "/car_state";
    m_target_vehicle_local_state_sub = m_nh.subscribe<uav_msgs::CarState>(m_car_state_sub_topic_name, 10, boost::bind(&GenerateWaypoints::TargetVehicleLocalStateCallback, this, _1));
    m_target_vehicle_global_position_sub = m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&GenerateWaypoints::TargetVehicleGlobalStateCallback, this, _1));

    // Initialize publisher
    m_input_local_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray>(self_pkg_name + self_node_name + "/input_waypoints", 1);
    m_desired_local_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray>(self_pkg_name + self_node_name + "/desired_local_waypoints", 1);
    m_global_to_enu_target_vehicle_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>(self_pkg_name + self_node_name + "/global_to_enu_target_vehicle_pose", 1);
    m_world_enu_desired_waypoints_pub = m_nh.advertise<geometry_msgs::PoseArray>(self_pkg_name + self_node_name + "/world_enu_desired_waypoints", 1);

    // Initialize service client
    m_desired_waypoints_srv_client = m_nh.serviceClient<uav_msgs::SetLocalPosition>("/airsim_node/local_position_goal");
}

void GenerateWaypoints::InitTargetVehicle()
{
    double dE = 0;
    double dN = 0;
    double K_Lon = 0;
    double K_Lat = 0;

    m_target_vehicle.global_pose_raw.pose.position.latitude = m_init_gps_lat_param;
    m_target_vehicle.global_pose_raw.pose.position.longitude = m_init_gps_lon_param;

    m_target_vehicle.global_pose_deg.pose.position.latitude = ((double)m_target_vehicle.global_pose_raw.pose.position.latitude)/10000000.0;
    m_target_vehicle.global_pose_rad.pose.position.latitude = m_target_vehicle.global_pose_deg.pose.position.latitude*M_PI/180.0;

    m_target_vehicle.global_pose_deg.pose.position.longitude = ((double)m_target_vehicle.global_pose_raw.pose.position.longitude)/10000000.0;
    m_target_vehicle.global_pose_rad.pose.position.longitude = m_target_vehicle.global_pose_deg.pose.position.longitude*M_PI/180.0;

    // init. position
    m_target_vehicle.east = 0.0;
    m_target_vehicle.north = 0.0;
    m_target_vehicle.prev_global_pose_rad.pose.position.latitude = m_target_vehicle.global_pose_rad.pose.position.latitude;
    m_target_vehicle.prev_global_pose_rad.pose.position.longitude = m_target_vehicle.global_pose_rad.pose.position.longitude;
    m_K_LON = 1/(m_utils.NormalRadius((double)SEMI_MAJOR_AXIS, (double)SEMI_MINOR_AXIS, m_target_vehicle.global_pose_rad.pose.position.latitude)*cos(m_target_vehicle.global_pose_rad.pose.position.latitude)); // const Kappa_lon
    m_K_LAT = 1/m_utils.MeridionalRadius((double)SEMI_MAJOR_AXIS, (double)SEMI_MINOR_AXIS, m_target_vehicle.global_pose_rad.pose.position.latitude);            // const Kappa_lat
}

void GenerateWaypoints::TargetVehicleLocalStateCallback(const uav_msgs::CarState::ConstPtr &car_state_ptr)
{
    geometry_msgs::PoseStamped pose;
    pose.header = car_state_ptr->header;
    pose.pose = car_state_ptr->pose.pose;

    if (!ConvertStateToLocalWaypoints(pose)) ROS_ERROR_STREAM("Fail convert state to pose");
}

void GenerateWaypoints::TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &current_pose_ptr)
{
    m_target_vehicle.global_pose_raw.pose.position.latitude = current_pose_ptr->latitude;
    m_target_vehicle.global_pose_raw.pose.position.longitude = current_pose_ptr->longitude;
    // gSpeed_raw = msg->gSpeed;
    // heading_raw = msg->heading;
    m_target_vehicle.heading_raw = current_pose_ptr->azimuth;

    m_target_vehicle.global_pose_deg.pose.position.latitude = ((double)m_target_vehicle.global_pose_raw.pose.position.latitude)/10000000.0;
    m_target_vehicle.global_pose_rad.pose.position.latitude = m_target_vehicle.global_pose_deg.pose.position.latitude*M_PI/180.0;

    m_target_vehicle.global_pose_deg.pose.position.longitude = ((double)m_target_vehicle.global_pose_raw.pose.position.longitude)/10000000.0;
    m_target_vehicle.global_pose_rad.pose.position.longitude = m_target_vehicle.global_pose_deg.pose.position.longitude*M_PI/180.0;

    // gSpeed = ((double)gSpeed_raw)/1000.0;

    m_target_vehicle.heading_deg = ((double)m_target_vehicle.heading_raw)/100000.0;
    m_target_vehicle.heading_rad = m_target_vehicle.heading_deg*M_PI/180.0;
    m_target_vehicle.heading_rad = -m_target_vehicle.heading_rad + M_PI/2;

    if (m_target_vehicle.heading_rad < 0){
        m_target_vehicle.heading_rad += 2*M_PI;
    }


    double dE = (m_target_vehicle.global_pose_rad.pose.position.longitude - m_target_vehicle.prev_global_pose_rad.pose.position.longitude)/m_K_LON;
    double dN = (m_target_vehicle.global_pose_rad.pose.position.altitude - m_target_vehicle.prev_global_pose_rad.pose.position.latitude)/m_K_LAT;
    m_target_vehicle.east += dE;
    m_target_vehicle.north += dN;

    m_target_vehicle.prev_global_pose_rad.pose.position.longitude = m_target_vehicle.global_pose_rad.pose.position.longitude;
    m_target_vehicle.prev_global_pose_rad.pose.position.latitude = m_target_vehicle.global_pose_rad.pose.position.latitude;
    
    m_target_vehicle.local_pose.header.frame_id = "world_enu";
    m_target_vehicle.local_pose.pose.position.x = m_target_vehicle.east;
    m_target_vehicle.local_pose.pose.position.y = m_target_vehicle.north;

    m_global_to_enu_target_vehicle_pose_pub.publish(m_target_vehicle.local_pose);
}

bool GenerateWaypoints::ConvertStateToLocalWaypoints(geometry_msgs::PoseStamped pose_stamped)
{
    bool do_push_waypoints = false;
    if (m_input_local_waypoints.size() < 2)
        do_push_waypoints = true;
    else
    {
        auto distance_m = m_utils.Distance(m_input_local_waypoints.back().pose.position, pose_stamped.pose.position);
        if (distance_m > m_distance_thresh_param)
            do_push_waypoints = true;
    }

    if (do_push_waypoints)
    {
        m_input_local_waypoints.push_back(pose_stamped);

        auto desired_local_pose = CreateDesiredLocalWaypoint(pose_stamped);
        m_desired_local_waypoints.push_back(desired_local_pose);

        uav_msgs::SetLocalPosition desired_local_position;
        desired_local_position.request.x = desired_local_pose.pose.position.x;
        desired_local_position.request.y = desired_local_pose.pose.position.y;
        desired_local_position.request.z = desired_local_pose.pose.position.z;
        auto euler = m_utils.Quat2Euler(desired_local_pose.pose.orientation);
        desired_local_position.request.yaw = euler.y;

        m_desired_waypoints_srv_client.call(desired_local_position);
    }
    else return true;

    geometry_msgs::PoseArray input_local_pose_array;
    input_local_pose_array.header = m_input_local_waypoints.back().header;

    for (auto p : m_input_local_waypoints) {
        input_local_pose_array.poses.push_back(p.pose);
    }

    geometry_msgs::PoseArray desired_local_pose_array;
    desired_local_pose_array.header = m_desired_local_waypoints.back().header;

    for (auto p : m_desired_local_waypoints) {
        desired_local_pose_array.poses.push_back(p.pose);
    }

    geometry_msgs::PoseArray world_enu_input_pose_array;
    // world_enu_input_pose_array = ConvertWorldEnu(input_local_pose_array);
    geometry_msgs::PoseArray world_enu_desired_pose_array;
    // world_enu_desired_pose_array = ConvertWorldEnu(desired_local_pose_array);

    m_input_local_waypoints_pub.publish(input_local_pose_array);
    m_desired_local_waypoints_pub.publish(desired_local_pose_array);

    return true;
}

geometry_msgs::PoseStamped GenerateWaypoints::CreateDesiredLocalWaypoint(geometry_msgs::PoseStamped pose_stamped)
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