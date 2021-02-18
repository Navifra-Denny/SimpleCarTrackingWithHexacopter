#include "control/tf_broadcaster.h"

TfBroadcaster::TfBroadcaster()
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
}

bool TfBroadcaster::GetParam()
{
    m_nh.getParam("tf_broadcaster_node/is_finding_home", m_is_finding_home_param);

    return true;
}

void TfBroadcaster::InitRos()
{
    // Initialize subscriber
    m_novatel_sub = m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&TfBroadcaster::NovatelINSPVACallback, this, _1));
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

void TfBroadcaster::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[ home set ]");
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
        geometry_msgs::PoseStamped novatel_enu_pose = 
            m_utils.ConvertToMapFrame(inspva_msg_ptr->latitude, inspva_msg_ptr->longitude, inspva_msg_ptr->height, m_home_position);
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