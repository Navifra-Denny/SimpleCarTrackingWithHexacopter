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
#include "uav_msgs/TargetWP.h"

class Eval{
public:
    Eval();
    ~Eval();

private:
    ros::NodeHandle m_nh;

    // initialize subscriber
    ros::Subscriber m_waypoints_sub;
    ros::Subscriber m_current_local_pose_sub;
    ros::Subscriber m_ego_vehicle_local_vel_sub;
    ros::Subscriber m_target_vehicle_global_position_sub;

    // initialize publisher
    ros::Publisher m_err_pos_pub;
    ros::Publisher m_err_vel_pub;
    ros::Publisher m_rmse_pub;

    // initialize timer
    ros::Timer m_eval_timer;

    // param

    // flag
    bool m_is_global;
    bool m_is_detected;
    bool m_is_rmse_started;

    control::Utils m_utils;
    control::VehicleState m_ego_vehicle;
    control::VehicleState m_target_vehicle;
    
    geometry_msgs::PoseStamped m_local_wp;
    geographic_msgs::GeoPoseStamped m_global_wp;
    unsigned int m_rmse_count;
    double m_sum_err_dist_sqrt;
    double m_rmse;

    void InitFlag();
    bool GetParam();
    void InitROS();

    void EvalTimerCallback(const ros::TimerEvent& event);
    
    void WaypointsCallback(const uav_msgs::TargetWP::ConstPtr &waypoints_ptr);
    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &local_position_ptr);
    void EgoVehicleLocalVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &twist_ptr);
    void TargetVehicleGlobalStateCallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr);
};

Eval::Eval()
{
    InitFlag();
    InitROS();
}

Eval::~Eval()
{}

void Eval::InitFlag()
{
    m_is_global = false;
    m_is_detected = false;
    m_is_rmse_started = false;
}

void Eval::InitROS()
{
    // Initialize subscriber
    m_waypoints_sub = m_nh.subscribe<uav_msgs::TargetWP>("/control/generate_waypoints_node/target_waypoints", 10, boost::bind(&Eval::WaypointsCallback, this, _1));
    m_current_local_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&Eval::EgoVehicleLocalPositionCallback, this, _1));
    m_ego_vehicle_local_vel_sub = m_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, boost::bind(&Eval::EgoVehicleLocalVelocityCallback, this, _1));
    m_target_vehicle_global_position_sub = 
        m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&Eval::TargetVehicleGlobalStateCallback, this, _1));

    // Initialize publisher
    m_err_pos_pub = m_nh.advertise<geometry_msgs::Point> ("/control/evaluation_node/err_pos", 10);
    m_err_vel_pub = m_nh.advertise<geometry_msgs::Point> ("/control/evaluation_node/err_vel", 10);
    m_rmse_pub = m_nh.advertise<std_msgs::Float64> ("/control/evaluation_node/rmse", 10);

    // Initialize timer
    m_eval_timer = m_nh.createTimer(ros::Duration(0.05), &Eval::EvalTimerCallback, this);
}

void Eval::EvalTimerCallback(const ros::TimerEvent& event)
{
    geometry_msgs::Point err_pos;
    if (!m_is_global){
        err_pos.x = m_ego_vehicle.local.pose.position.x - m_local_wp.pose.position.x;
        err_pos.y = m_ego_vehicle.local.pose.position.y - m_local_wp.pose.position.y;
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
            m_sum_err_dist_sqrt = 0;
        }
        m_rmse_count++;
        m_sum_err_dist_sqrt += err_dist.x;
        m_rmse = sqrt(m_sum_err_dist_sqrt/(double)m_rmse_count);
    }
    else {
        m_is_rmse_started = false;
        m_rmse_count = 0;
        m_sum_err_dist_sqrt = 0;
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

void Eval::WaypointsCallback(const uav_msgs::TargetWP::ConstPtr &waypoints_ptr)
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
    m_target_vehicle.velocity.linear.x = inspva_msg_ptr->east_velocity;
    m_target_vehicle.velocity.linear.y = inspva_msg_ptr->north_velocity;
}


int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "evaluation_node");
    Eval eval;

    ros::spin();
    return 0;
}