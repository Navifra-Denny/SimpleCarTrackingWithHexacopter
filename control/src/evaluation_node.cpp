#include "ros/ros.h"
#include <ros/spinner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>

#include <geographic_msgs/GeoPath.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <std_msgs/Float64.h>

#include <novatel_oem7_msgs/INSPVA.h>
#include "control/generate_waypoints.h"
#include "uav_msgs/TargetWaypoints.h"
#include "mavros_msgs/HomePosition.h"

class Eval{
public:
    Eval();
    ~Eval();

private:
    ros::NodeHandle m_nh;

    // initialize subscriber
    ros::Subscriber m_waypoints_sub;
    ros::Subscriber m_ego_local_pose_sub;
    ros::Subscriber m_ego_local_vel_sub;
    ros::Subscriber m_gps_based_target_global_position_sub;
    ros::Subscriber m_home_position_sub;

    // initialize publisher
    ros::Publisher m_err_pos_pub;
    ros::Publisher m_err_vel_pub;
    ros::Publisher m_rmse_pub;

    // initialize timer
    ros::Timer m_eval_timer;

    // param
    double m_target_height_m_param;
    bool m_is_home_set;

    // flag
    bool m_is_global;
    bool m_is_detected;
    bool m_is_rmse_started;

    control::Utils m_utils;
    control::VehicleState m_ego_vehicle;
    control::VehicleState m_target_vehicle;
    
    geographic_msgs::GeoPoint m_home_position;
    geometry_msgs::PoseStamped m_local_wp;
    geographic_msgs::GeoPoseStamped m_global_wp;
    unsigned int m_rmse_count;
    double m_err_sum_dist;
    double m_rmse;

    void InitFlag();
    bool GetParam();
    void InitROS();

    void EvalTimerCallback(const ros::TimerEvent& event);
    
    void WaypointsCallback(const uav_msgs::TargetWaypoints::ConstPtr &waypoints_ptr);
    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &local_position_ptr);
    void EgoVehicleLocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &twist_ptr);
    void TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr);
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
};

Eval::Eval() :
    m_target_height_m_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();
}

Eval::~Eval()
{}

void Eval::InitFlag()
{
    m_is_global = false;
    m_is_detected = false;
    m_is_rmse_started = false;
    m_is_home_set = false;
}

bool Eval::GetParam()
{
	m_nh.getParam("evaluation_node/target_height_m", m_target_height_m_param);
    
    if (__isnan(m_target_height_m_param)) { ROS_ERROR_STREAM("m_target_height_m_param is NAN"); return false; }

    return true;
}

void Eval::InitROS()
{
    // Initialize subscriber
    m_waypoints_sub = m_nh.subscribe<uav_msgs::TargetWaypoints>("/control/generate_waypoints_node/target_waypoints", 10, boost::bind(&Eval::WaypointsCallback, this, _1));
    m_ego_local_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&Eval::EgoVehicleLocalPositionCallback, this, _1));
    m_ego_local_vel_sub = m_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, boost::bind(&Eval::EgoVehicleLocalVelocityCallback, this, _1));
    m_gps_based_target_global_position_sub = 
        m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&Eval::TargetVehicleGlobalStateCallback, this, _1));
    m_home_position_sub = 
        m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&Eval::HomePositionCallback, this, _1));

    ros::Rate rate(10);
    while (ros::ok() && !m_is_home_set){
        ros::spinOnce();
        rate.sleep();
    }
    
    // Initialize publisher
    m_err_pos_pub = m_nh.advertise<geometry_msgs::Point> ("/control/evaluation_node/err_pos", 10);
    m_err_vel_pub = m_nh.advertise<geometry_msgs::Point> ("/control/evaluation_node/err_vel", 10);
    m_rmse_pub = m_nh.advertise<std_msgs::Float64> ("/control/evaluation_node/rmse", 10);

    // Initialize timer
    m_eval_timer = m_nh.createTimer(ros::Duration(0.1), &Eval::EvalTimerCallback, this);
}

void Eval::EvalTimerCallback(const ros::TimerEvent& event)
{
    geometry_msgs::Point err_pos;
    if (!m_is_global){
        err_pos.x = m_ego_vehicle.local.pose.position.x - m_target_vehicle.local.pose.position.x;
        err_pos.y = m_ego_vehicle.local.pose.position.y - m_target_vehicle.local.pose.position.y;
        err_pos.z = m_ego_vehicle.local.pose.position.z - m_local_wp.pose.position.z;
    }
    
    geometry_msgs::Point err_vel;
    if (!m_is_global){
        err_vel.x = m_ego_vehicle.velocity.linear.x - m_target_vehicle.velocity.linear.x;
        err_vel.y = m_ego_vehicle.velocity.linear.y - m_target_vehicle.velocity.linear.y;
    }

    geometry_msgs::Point err_dist;
    if (!m_is_global){
        err_dist.x = m_utils.Size(err_pos.x, err_pos.y);
    }

    if (m_is_detected){
        if (!m_is_rmse_started){
            m_is_rmse_started = true;
            m_rmse_count = 0;
            m_err_sum_dist = 0;
        }
        m_rmse_count++;
        m_err_sum_dist += err_dist.x;
        m_rmse = m_err_sum_dist/(double)m_rmse_count;
    }
    else {
        m_is_rmse_started = false;
        m_rmse_count = 0;
        m_err_sum_dist = 0;
    }

    std_msgs::Float64 rmse;
    rmse.data = m_rmse;
    m_err_pos_pub.publish(err_pos);
    m_err_vel_pub.publish(err_vel);
    m_rmse_pub.publish(rmse);
}

void Eval::EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &local_position_ptr)
{
    m_ego_vehicle.local.pose = local_position_ptr->pose;
}

void Eval::EgoVehicleLocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &twist_ptr)
{
    m_ego_vehicle.velocity.linear.x = twist_ptr->twist.linear.x;
    m_ego_vehicle.velocity.linear.y = twist_ptr->twist.linear.y;
    m_ego_vehicle.velocity.linear.z = twist_ptr->twist.linear.z;
}

void Eval::WaypointsCallback(const uav_msgs::TargetWaypoints::ConstPtr &waypoints_ptr)
{
    m_is_global = waypoints_ptr->state.is_global;
    m_is_detected = waypoints_ptr->state.is_detected;

    if (m_is_global){
        if (waypoints_ptr->global.poses.size() != 0){
            m_global_wp = waypoints_ptr->global.poses.back();
        }
    }
    else{
        if (waypoints_ptr->local.poses.size() != 0){
            m_local_wp.pose = waypoints_ptr->local.poses.back();
        }
    }
}

void Eval::TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr)
{
    geographic_msgs::GeoPoint geopoint;
    geopoint.latitude = inspva_msg_ptr->latitude;
    geopoint.longitude = inspva_msg_ptr->longitude;
    geopoint.altitude = inspva_msg_ptr->height;
        
    geometry_msgs::PoseStamped posestamped;
    if (!m_utils.IsNan(geopoint)){
        posestamped = m_utils.ConvertToMapFrame(geopoint.latitude, 
                                                geopoint.longitude, 
                                                m_target_height_m_param,
                                                m_home_position);
        
        tf2::Quaternion q;
        q.setRPY(inspva_msg_ptr->roll * M_PI / 180., inspva_msg_ptr->pitch * M_PI / 180., (-1*inspva_msg_ptr->azimuth + 90.0) * M_PI / 180.);
        posestamped.pose.orientation.x = q.x();
        posestamped.pose.orientation.y = q.y();
        posestamped.pose.orientation.z = q.z();
        posestamped.pose.orientation.w = q.w();
    }

    m_target_vehicle.local.pose = posestamped.pose;

    m_target_vehicle.velocity.linear.x = inspva_msg_ptr->east_velocity;
    m_target_vehicle.velocity.linear.y = inspva_msg_ptr->north_velocity;
}


void Eval::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[evaulation_node] Home set");
    }
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "evaluation_node");
    Eval eval;

    ros::spin();
    return 0;
}