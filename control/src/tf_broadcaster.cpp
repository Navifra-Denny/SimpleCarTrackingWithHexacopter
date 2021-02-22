#include "control/tf_broadcaster.h"
#include <math.h>

TfBroadcaster::TfBroadcaster() :
    m_target_height_m_param(NAN),
    G(9.80665)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitRos();
}

TfBroadcaster::~TfBroadcaster()
{}

void TfBroadcaster::InitFlag()
{
    m_is_home_set = false;
    m_is_first_imu = true;
}

bool TfBroadcaster::GetParam()
{
    m_nh.getParam("tf_broadcaster_node/is_finding_home", m_is_finding_home_param);
    m_nh.getParam("generate_waypoints_node/target_height_m", m_target_height_m_param);

    if (__isnan(m_target_height_m_param)) { ROS_ERROR_STREAM("m_target_height_m_param is NAN"); return false; }

    return true;
}

void TfBroadcaster::InitRos()
{
    // Initialize subscriber
    m_novatel_sub = m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&TfBroadcaster::NovatelINSPVACallback, this, _1));
    m_ego_vehicle_local_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&TfBroadcaster::EgoVehicleLocalPositionCallback, this, _1));
    // m_imu_sub = m_nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, boost::bind(&TfBroadcaster::EgoVehicleImuCallback, this, _1));
    if (!m_is_finding_home_param){
        m_home_position_sub = m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&TfBroadcaster::HomePositionCallback, this, _1));
        
        ros::Rate rate(10);
        while (ros::ok() && !m_is_home_set){
            ros::spinOnce();
            rate.sleep();
        }
    }
    // Initialize publisher
    m_home_position_pub = m_nh.advertise<geographic_msgs::GeoPoint>("/control/tf_broadcaster_node/home", 1);

    // Time callback
    m_home_position_timer = m_nh.createTimer(ros::Duration(2.0), &TfBroadcaster::HomePositionTimerCallback, this);
}

void TfBroadcaster::HomePositionTimerCallback(const ros::TimerEvent& event)
{
    if (m_is_home_set){
        m_home_position_pub.publish(m_home_position);
    }
}

void TfBroadcaster::NovatelINSPVACallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr)
{
    if (m_is_home_set){
        static tf2_ros::TransformBroadcaster novatel_tf_broadcaster;
        geometry_msgs::TransformStamped novatel_tf_stamped;

        novatel_tf_stamped.header.stamp = inspva_msg_ptr->header.stamp;
        novatel_tf_stamped.header.frame_id = "map";
        novatel_tf_stamped.child_frame_id = inspva_msg_ptr->header.frame_id;

        // Get offset
        geometry_msgs::PoseStamped novatel_enu_pose = m_utils.ConvertToMapFrame(inspva_msg_ptr->latitude, 
                                                                                inspva_msg_ptr->longitude, 
                                                                                m_target_height_m_param, 
                                                                                m_home_position);
        novatel_tf_stamped.transform.translation.x = novatel_enu_pose.pose.position.x;
        novatel_tf_stamped.transform.translation.y = novatel_enu_pose.pose.position.y;
        novatel_tf_stamped.transform.translation.z = novatel_enu_pose.pose.position.z;
        
        tf2::Quaternion q;
        q.setRPY(inspva_msg_ptr->roll * M_PI / 180., inspva_msg_ptr->pitch * M_PI / 180., (-1*inspva_msg_ptr->azimuth + 90.0) * M_PI / 180.);
        novatel_tf_stamped.transform.rotation.x = q.x();
        novatel_tf_stamped.transform.rotation.y = q.y();
        novatel_tf_stamped.transform.rotation.z = q.z();
        novatel_tf_stamped.transform.rotation.w = q.w();

        novatel_tf_broadcaster.sendTransform(novatel_tf_stamped);
    }
    else if (!m_is_home_set && m_is_finding_home_param){
        m_is_home_set = true;

        m_home_position.latitude = inspva_msg_ptr->latitude;
        m_home_position.longitude = inspva_msg_ptr->longitude;
        m_home_position.altitude = inspva_msg_ptr->height;
    }
}

void TfBroadcaster::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[tf_broadcaster] Home set");
    }
}

void TfBroadcaster::EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_stamped_ptr)
{
    static tf2_ros::TransformBroadcaster odom_tf_broadcaster;
    geometry_msgs::TransformStamped odom_tf_stamped;

    odom_tf_stamped.header.stamp = pose_stamped_ptr->header.stamp;
    odom_tf_stamped.header.frame_id = pose_stamped_ptr->header.frame_id;
    odom_tf_stamped.child_frame_id = "base_link";

    // Get offset
    odom_tf_stamped.transform.translation.x = pose_stamped_ptr->pose.position.x;
    odom_tf_stamped.transform.translation.y = pose_stamped_ptr->pose.position.y;
    odom_tf_stamped.transform.translation.z = pose_stamped_ptr->pose.position.z;
    
    tf2::Quaternion q;
    odom_tf_stamped.transform.rotation.x = pose_stamped_ptr->pose.orientation.x;
    odom_tf_stamped.transform.rotation.y = pose_stamped_ptr->pose.orientation.y;
    odom_tf_stamped.transform.rotation.z = pose_stamped_ptr->pose.orientation.z;
    odom_tf_stamped.transform.rotation.w = pose_stamped_ptr->pose.orientation.w;

    odom_tf_broadcaster.sendTransform(odom_tf_stamped);

    geometry_msgs::TransformStamped lidar_tf_stamped;
    lidar_tf_stamped.header.stamp = pose_stamped_ptr->header.stamp;
    lidar_tf_stamped.header.frame_id = "base_link";
    lidar_tf_stamped.child_frame_id = "velodyne";

    // Get offset
    lidar_tf_stamped.transform.translation.x = 0.0;
    lidar_tf_stamped.transform.translation.y = 0.0;
    lidar_tf_stamped.transform.translation.z = -0.5;

    auto euler = m_utils.Quat2Euler(pose_stamped_ptr->pose.orientation);
    
    double roll_rad = -euler.r;
    double pitch_rad = M_PI/2.0 - euler.p; // m_utils.Deg2Rad(-pitch_deg);
    double yaw_rad = -(M_PI/2.0) + euler.y;

    
    
    // double roll_deg = m_utils.Rad2Deg(euler.r);
    // double pitch_deg = m_utils.Rad2Deg(90 - euler.p);
    // double yaw_deg = m_utils.Rad2Deg(euler.y);
    
    
    // ROS_ERROR("r: 0.0, p: %f, y: %f", euler.p, euler.y);

    // double roll_rad = 0.0;
    // double pitch_rad = M_PI/2.0;// - euler.p; // m_utils.Deg2Rad(-pitch_deg);
    // double yaw_rad = -M_PI/2.0;

    // double roll_deg = m_utils.Rad2Deg(euler.r);
    // double pitch_deg = m_utils.Rad2Deg(euler.p);
    // double yaw_deg = m_utils.Rad2Deg(euler.y);

    // double roll_rad = m_utils.Deg2Rad(0.0 -  - roll_deg);
    // double pitch_rad = m_utils.Deg2Rad(90.0 - m_init_pitch_rad - pitch_deg);
    // double yaw_rad = m_utils.Deg2Rad(-90.0 + yaw_deg);

    // double roll_rad = m_init_roll_rad - euler.r;
    // double pitch_rad = (M_PI / 2.0) - euler.p - m_init_pitch_rad ;
    // double yaw_rad = -(M_PI / 2.0) + euler.y;

    q.setRPY(roll_rad, pitch_rad, yaw_rad);
    lidar_tf_stamped.transform.rotation.x = q.x();
    lidar_tf_stamped.transform.rotation.y = q.y();
    lidar_tf_stamped.transform.rotation.z = q.z();
    lidar_tf_stamped.transform.rotation.w = q.w();

    odom_tf_broadcaster.sendTransform(lidar_tf_stamped);

    // ROS_ERROR("r: %f, p: %f, y: %f", m_utils.Rad2Deg(euler.r), m_utils.Rad2Deg(euler.p), m_utils.Rad2Deg(euler.y));
    // ROS_ERROR("r: %f, p: %f, y: %f", euler.r, euler.p, euler.y);
    // ROS_WARN("r: %f, p: %f, y: %f", m_utils.Rad2Deg(euler.r), m_utils.Rad2Deg(euler.p), m_utils.Rad2Deg(euler.y));
}

// void TfBroadcaster::EgoVehicleImuCallback(const sensor_msgs::Imu::ConstPtr &imu_ptr)
// {
//     if (m_is_first_imu){
//         m_is_first_imu = false;

//         auto a_y = imu_ptr->linear_acceleration.y;
//         auto a_x = imu_ptr->linear_acceleration.x;

//         m_init_pitch_rad = atan2(a_x, G);
//         m_init_roll_rad = asin(a_y/G);
//     }
// }
