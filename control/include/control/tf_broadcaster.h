#ifndef __TF_BROADCASTER_H__
#define __TF_BROADCASTER_H__


#include "ros/ros.h"
#include <ros/spinner.h>

#include <mavros_msgs/HomePosition.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <novatel_oem7_msgs/INSPVA.h>

#include <sensor_msgs/Imu.h>

#include "control/utils.h"

class TfBroadcaster{
private:
	ros::NodeHandle m_nh;
    control::Utils m_utils;

public:
    TfBroadcaster();
    ~TfBroadcaster();

private:
    // subscriber
    ros::Subscriber m_novatel_sub;
    ros::Subscriber m_ego_vehicle_local_pose_sub;
    ros::Subscriber m_ego_vehicle_imu_sub;
    ros::Subscriber m_home_position_sub;

    // publisher
    ros::Publisher m_home_position_pub;

    // timer
    ros::Timer m_home_position_timer;

    // param
    bool m_is_finding_home_param;
    bool m_target_height_m_param;
    
    // time
    ros::Time m_prev_imu_time;

    geographic_msgs::GeoPoint m_home_position;
    bool m_is_home_set;
    control::Euler m_ego_vehicle_attitude;
    geometry_msgs::Point m_ego_vehicle_position;

    tf2_ros::StaticTransformBroadcaster m_odom_static_tf_broadcaster;

private:
    void InitFlag();
    bool GetParam();
    void InitRos();
    void InitStaticTf();

    void HomePositionTimerCallback(const ros::TimerEvent& event);

    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
    void NovatelINSPVACallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr);
    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_stamped_ptr);
    void EgoVehicleImuCallback(const sensor_msgs::Imu::ConstPtr &imu_ptr);
};





#endif // __TF_BROADCASTER_H__