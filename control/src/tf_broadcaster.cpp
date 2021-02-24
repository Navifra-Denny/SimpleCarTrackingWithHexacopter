#include "control/tf_broadcaster.h"
#include <math.h>
#include <Eigen/Dense>

TfBroadcaster::TfBroadcaster() :
    m_target_height_m_param(NAN),
    m_prev_imu_time(ros::Time(0))
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitRos();
    InitStaticTf();

    m_ego_vehicle_attitude.r = 0.0;
    m_ego_vehicle_attitude.p = 0.0;
    m_ego_vehicle_attitude.y = 0.0;
}

TfBroadcaster::~TfBroadcaster()
{}

void TfBroadcaster::InitFlag()
{
    m_is_home_set = false;
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
    m_ego_vehicle_imu_sub = m_nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, boost::bind(&TfBroadcaster::EgoVehicleImuCallback, this, _1));
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

void TfBroadcaster::InitStaticTf(void)
{
    static tf2_ros::StaticTransformBroadcaster odom_static_tf_broadcaster;
    geometry_msgs::TransformStamped odom_tf_stamped;

    odom_tf_stamped.header.stamp = ros::Time::now();
    odom_tf_stamped.header.frame_id = "map_ned";
    odom_tf_stamped.child_frame_id = "odom";

    odom_tf_stamped.transform.translation.x = 0.0;
    odom_tf_stamped.transform.translation.y = 0.0;
    odom_tf_stamped.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    odom_tf_stamped.transform.rotation.x = 0.0;
    odom_tf_stamped.transform.rotation.y = 0.0;
    odom_tf_stamped.transform.rotation.z = 0.0;
    odom_tf_stamped.transform.rotation.w = 1.0;

    odom_static_tf_broadcaster.sendTransform(odom_tf_stamped);
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
    // Get offset
    m_ego_vehicle_position.x = pose_stamped_ptr->pose.position.x;
    m_ego_vehicle_position.y = pose_stamped_ptr->pose.position.y;
    m_ego_vehicle_position.z = pose_stamped_ptr->pose.position.z;

    static tf2_ros::TransformBroadcaster lidar_tf_broadcaster;
    geometry_msgs::TransformStamped lidar_tf_stamped;
    lidar_tf_stamped.header.stamp = pose_stamped_ptr->header.stamp;
    lidar_tf_stamped.header.frame_id = "base_link";
    lidar_tf_stamped.child_frame_id = "velodyne";

    // Get offset
    lidar_tf_stamped.transform.translation.x = 0.0;
    lidar_tf_stamped.transform.translation.y = 0.0;
    lidar_tf_stamped.transform.translation.z = -0.5;
    
    tf2::Quaternion q;
    q.setRPY(0.0, M_PI/2.0, M_PI/2.0);
    lidar_tf_stamped.transform.rotation.x = q.x();
    lidar_tf_stamped.transform.rotation.y = q.y();
    lidar_tf_stamped.transform.rotation.z = q.z();
    lidar_tf_stamped.transform.rotation.w = q.w();

    lidar_tf_broadcaster.sendTransform(lidar_tf_stamped);
}


void TfBroadcaster::EgoVehicleImuCallback(const sensor_msgs::Imu::ConstPtr &imu_ptr)
{
    ros::Time curr_time = ros::Time::now();
    auto dt = curr_time - m_prev_imu_time;
    m_prev_imu_time = curr_time;

    static tf2_ros::TransformBroadcaster base_link_tf_broadcaster;
    geometry_msgs::TransformStamped base_link_tf_stamped;
    base_link_tf_stamped.header.stamp = imu_ptr->header.stamp;
    base_link_tf_stamped.header.frame_id = "map";
    base_link_tf_stamped.child_frame_id = "base_link";

    // Get offset
    base_link_tf_stamped.transform.translation.x = m_ego_vehicle_position.x;
    base_link_tf_stamped.transform.translation.y = m_ego_vehicle_position.y;
    base_link_tf_stamped.transform.translation.z = m_ego_vehicle_position.z;

    auto p = imu_ptr->angular_velocity.x;
    auto q = imu_ptr->angular_velocity.y;
    auto r = imu_ptr->angular_velocity.z;
    Eigen::Vector3d body_rads;
    body_rads << p, q, r;

    auto body_euler = m_utils.Quat2Euler(imu_ptr->orientation);
    auto enu_rads = m_utils.BodyRads2EnuRads(body_euler, body_rads);
    Eigen::Vector3d enu_euler = enu_rads * dt.toSec();

    m_ego_vehicle_attitude.r -= enu_euler(0);
    m_ego_vehicle_attitude.p -= enu_euler(1);
    m_ego_vehicle_attitude.y -= enu_euler(2);

    tf2::Quaternion quat;
    quat.setRPY(m_ego_vehicle_attitude.r, m_ego_vehicle_attitude.p, m_ego_vehicle_attitude.y);
    base_link_tf_stamped.transform.rotation.x = quat.x();
    base_link_tf_stamped.transform.rotation.y = quat.y();
    base_link_tf_stamped.transform.rotation.z = quat.z();
    base_link_tf_stamped.transform.rotation.w = quat.w();

    base_link_tf_broadcaster.sendTransform(base_link_tf_stamped);
}