#include "ros/ros.h"
#include <ros/spinner.h>

#include "uav_msgs/GenerateWaypointState.h"
#include "uav_msgs/OffboardState.h"
#include "uav_msgs/ControlStatus.h"

class Checker{
public:
    Checker();
    ~Checker();

private:
    ros::NodeHandle m_nh;

    // Initialize subscriber
    ros::Subscriber m_gen_wp_state_sub;
    ros::Subscriber m_offb_state_sub;

    // Initialize publisher
    ros::Publisher m_control_status_pub;

    // Initialize timer
    ros::Timer m_checker_timer;

    uav_msgs::ControlStatus m_control_status;

    bool GetParam();
    void InitROS();

    void CheckerTimerCallback(const ros::TimerEvent& event);
    
    void GenWpStateCallback(const uav_msgs::GenerateWaypointState::ConstPtr &gen_wp_state_ptr);
    void OffbStateCallback(const uav_msgs::OffboardState::ConstPtr &offb_state_ptr);
};

Checker::Checker()
{
    InitROS();
}

Checker::~Checker()
{}

void Checker::InitROS()
{
    // package, node, topic name
    std::string node_name_with_namespace = ros::this_node::getName();

    // Initialize subscriber
    m_gen_wp_state_sub = 
        m_nh.subscribe<uav_msgs::GenerateWaypointState>("/control/generate_waypoints_node/gen_wp_state", 10, boost::bind(&Checker::GenWpStateCallback, this, _1));
    m_offb_state_sub = 
        m_nh.subscribe<uav_msgs::OffboardState>("/control/mavros_offb_node/offboard_state", 10, boost::bind(&Checker::OffbStateCallback, this, _1));

    // Initialize publisher
    m_control_status_pub = m_nh.advertise<uav_msgs::ControlStatus>(node_name_with_namespace + "/uav_status", 10);

    // Initialize timer
    m_checker_timer = m_nh.createTimer(ros::Duration(0.1), &Checker::CheckerTimerCallback, this);
}


void Checker::GenWpStateCallback(const uav_msgs::GenerateWaypointState::ConstPtr &gen_wp_state_ptr)
{
    m_control_status.generate_waypoint_state.is_detected = gen_wp_state_ptr->is_detected;
    m_control_status.generate_waypoint_state.is_hover = gen_wp_state_ptr->is_hover;
    m_control_status.generate_waypoint_state.is_reached = gen_wp_state_ptr->is_reached;
    m_control_status.generate_waypoint_state.is_global = gen_wp_state_ptr->is_global;
    m_control_status.generate_waypoint_state.global_to_local = gen_wp_state_ptr->global_to_local;
    m_control_status.generate_waypoint_state.detection_tool = gen_wp_state_ptr->detection_tool;
    m_control_status.generate_waypoint_state.curr_local_position = gen_wp_state_ptr->curr_local_position;
    m_control_status.generate_waypoint_state.curr_global_position = gen_wp_state_ptr->curr_global_position;
    m_control_status.generate_waypoint_state.local_waypoint_position = gen_wp_state_ptr->local_waypoint_position;
    m_control_status.generate_waypoint_state.global_waypoint_position = gen_wp_state_ptr->global_waypoint_position;
}

void Checker::OffbStateCallback(const uav_msgs::OffboardState::ConstPtr &offb_state_ptr)
{
    m_control_status.offboard_state.px4_mode = offb_state_ptr->px4_mode;
    m_control_status.offboard_state.is_debug_mode = offb_state_ptr->is_debug_mode;
}

void Checker::CheckerTimerCallback(const ros::TimerEvent& event)
{
    m_control_status_pub.publish(m_control_status);
}


int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "checker_node");
    Checker checker;

    ros::spin();
    return 0;
}