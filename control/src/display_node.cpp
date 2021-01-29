#include "ros/ros.h"
#include <ros/spinner.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include "uav_msgs/PolyfitLane.h"
#include "uav_msgs/CarState.h"
#include "nav_msgs/Odometry.h"
#include "uav_msgs/Roi.h"


struct Line{
    visualization_msgs::MarkerArray self;
    visualization_msgs::Marker line_strip;
    visualization_msgs::Marker orientation;
    visualization_msgs::Marker center_point;
    visualization_msgs::Marker roi;
};

class Display
{
public:
    Display();
    virtual ~Display();
private:
    // Node Handler
	ros::NodeHandle m_nh;

	// subscriber
	ros::Subscriber m_input_waypoints_sub;
	ros::Subscriber m_input_curr_position_sub;
	ros::Subscriber m_desired_waypoints_sub;
    ros::Subscriber m_roi_waypoints_sub;
    ros::Subscriber m_roi_curr_position_sub;
    ros::Subscriber m_roi_box_sub;
    
    // publisher
    ros::Publisher m_input_line_pub;
    ros::Publisher m_desired_line_pub;
    ros::Publisher m_roi_line_pub;
    ros::Timer m_roi_line_update_timer;

    // Parm
    std::string m_ugv_name_param;
    std::string m_uav_name_param;

    // Marker Init
    void SetParam();
    void MarkerInit();

    // Callback
    void InputWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array);
    void InputPositionCallback(const uav_msgs::CarState::ConstPtr &current_point_ptr);
    void DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array);
    void ROIWaypointsCallback(const uav_msgs::PolyfitLane::ConstPtr &roi_waypoints);
    void ROICurrPositionCallback(const nav_msgs::Odometry::ConstPtr &current_point_ptr);
    void ROICallback(const uav_msgs::Roi::ConstPtr &current_point_ptr);
    void TimerCallback(const ros::TimerEvent& event);

    Line m_input_line;
    Line m_desired_line;
    Line m_roi_line;
    // visualization_msgs::MarkerArray desired_line;
    // visualization_msgs::Marker line_strip;

private:
    void GenerateDesiredWaypoints();
};

Display::Display()
{
    SetParam();
    MarkerInit();

    // topic name
    std::string input_curr_position_sub_topic_name = "/airsim_node/" + m_ugv_name_param + "/car_state";
	std::string roi_curr_position_sub_topic_name = "/airsim_node/" + m_uav_name_param + "/odom_local_ned";

    // subscriber init
	m_input_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>("/generate_waypoints_node/input_waypoints", 1, boost::bind(&Display::InputWaypointsCallback, this, _1));
	m_input_curr_position_sub = m_nh.subscribe<uav_msgs::CarState>(input_curr_position_sub_topic_name, 10, boost::bind(&Display::InputPositionCallback, this, _1));
	m_desired_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>("/generate_waypoints_node/desired_waypoints", 1, boost::bind(&Display::DesiredWaypointsCallback, this, _1));
	m_roi_waypoints_sub = m_nh.subscribe<uav_msgs::PolyfitLane>("/extract_lane_node/poly_fit_lane", 1, boost::bind(&Display::ROIWaypointsCallback, this, _1));
	m_roi_curr_position_sub = m_nh.subscribe<nav_msgs::Odometry>(roi_curr_position_sub_topic_name, 1, boost::bind(&Display::ROICurrPositionCallback, this, _1));
	m_roi_box_sub = m_nh.subscribe<uav_msgs::Roi>("/extract_lane_node/roi", 1, boost::bind(&Display::ROICallback, this, _1));

    // publisher init
    // m_input_line_pub = m_nh.advertise<visualization_msgs::Marker> ("/Marker/input_line", 1);
    m_input_line_pub = m_nh.advertise<visualization_msgs::MarkerArray> ("/Marker/input_line", 1);
    m_desired_line_pub = m_nh.advertise<visualization_msgs::MarkerArray> ("/Marker/desired_line", 1);
    m_roi_line_pub = m_nh.advertise<visualization_msgs::MarkerArray> ("/Marker/roi_line", 1);

    m_roi_line_update_timer = m_nh.createTimer(ros::Duration(0.1), &Display::TimerCallback, this);
}

Display::~Display()
{}

void Display::SetParam()
{
	m_nh.getParam("display_node/ugv_name", m_ugv_name_param);
	m_nh.getParam("display_node/ugv_name", m_uav_name_param);
}

void Display::MarkerInit()
{
    //// input line
    m_input_line.line_strip.ns = "lane";
    m_input_line.line_strip.id = 0;
    m_input_line.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_input_line.line_strip.action = visualization_msgs::Marker::ADD;
    m_input_line.line_strip.scale.x = 0.05;
    m_input_line.line_strip.pose.orientation.w = 1.0;
    m_input_line.line_strip.pose.orientation.x = 0.0;
    m_input_line.line_strip.pose.orientation.y = 0.0;
    m_input_line.line_strip.pose.orientation.z = 0.0;
    m_input_line.line_strip.color.r = 0.0f;
    m_input_line.line_strip.color.g = 1.0f;
    m_input_line.line_strip.color.b = 0.0f;
    m_input_line.line_strip.color.a = 1.0;
    m_input_line.line_strip.lifetime = ros::Duration();

    m_input_line.orientation.ns = "orientation";
    m_input_line.orientation.id = 0;
    m_input_line.orientation.type = visualization_msgs::Marker::ARROW;
    m_input_line.orientation.action = visualization_msgs::Marker::ADD;
    m_input_line.orientation.scale.x = 1.0;
    m_input_line.orientation.scale.y = 0.2;
    m_input_line.orientation.scale.z = 0.2;
    m_input_line.orientation.color.r = 1.0f;
    m_input_line.orientation.color.g = 0.0f;
    m_input_line.orientation.color.b = 0.0f;
    m_input_line.orientation.color.a = 1.0;
    m_input_line.orientation.lifetime = ros::Duration();

    m_input_line.center_point.ns = "center_point";
    m_input_line.center_point.id = 0;
    m_input_line.center_point.type = visualization_msgs::Marker::SPHERE;
    m_input_line.center_point.action = visualization_msgs::Marker::ADD;
    m_input_line.center_point.scale.x = 0.3;
    m_input_line.center_point.scale.y = 0.3;
    m_input_line.center_point.scale.z = 0.3;
    m_input_line.center_point.pose.orientation.w = 1.0;
    m_input_line.center_point.pose.orientation.x = 0.0;
    m_input_line.center_point.pose.orientation.y = 0.0;
    m_input_line.center_point.pose.orientation.z = 0.0;
    m_input_line.center_point.color.r = 0.0f;
    m_input_line.center_point.color.g = 1.0f;
    m_input_line.center_point.color.b = 0.0f;
    m_input_line.center_point.color.a = 1.0;
    m_input_line.center_point.lifetime = ros::Duration();

    //// desired line
    m_desired_line.line_strip.ns = "lane";
    m_desired_line.line_strip.id = 0;
    m_desired_line.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_desired_line.line_strip.action = visualization_msgs::Marker::ADD;
    m_desired_line.line_strip.scale.x = 0.05;
    m_desired_line.line_strip.pose.orientation.w = 1.0;
    m_desired_line.line_strip.pose.orientation.x = 0.0;
    m_desired_line.line_strip.pose.orientation.y = 0.0;
    m_desired_line.line_strip.pose.orientation.z = 0.0;
    m_desired_line.line_strip.color.r = 1.0f;
    m_desired_line.line_strip.color.g = 0.0f;
    m_desired_line.line_strip.color.b = 0.0f;
    m_desired_line.line_strip.color.a = 1.0;
    m_desired_line.line_strip.lifetime = ros::Duration();

    //// roi line
    m_roi_line.line_strip.ns = "lane";
    m_roi_line.line_strip.id = 0;
    m_roi_line.line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    m_roi_line.line_strip.action = visualization_msgs::Marker::ADD;
    m_roi_line.line_strip.scale.x = 0.1;
    m_roi_line.line_strip.pose.orientation.w = 1.0;
    m_roi_line.line_strip.pose.orientation.x = 0.0;
    m_roi_line.line_strip.pose.orientation.y = 0.0;
    m_roi_line.line_strip.pose.orientation.z = 0.0;
    m_roi_line.line_strip.color.r = 0.5f;
    m_roi_line.line_strip.color.g = 0.5f;
    m_roi_line.line_strip.color.b = 0.0f;
    m_roi_line.line_strip.color.a = 1.0;
    m_roi_line.line_strip.lifetime = ros::Duration();

    m_roi_line.center_point.ns = "center_point";
    m_roi_line.center_point.id = 0;
    m_roi_line.center_point.type = visualization_msgs::Marker::SPHERE;
    m_roi_line.center_point.action = visualization_msgs::Marker::ADD;
    m_roi_line.center_point.scale.x = 0.3;
    m_roi_line.center_point.scale.y = 0.3;
    m_roi_line.center_point.scale.z = 0.3;
    m_roi_line.center_point.pose.orientation.w = 1.0;
    m_roi_line.center_point.pose.orientation.x = 0.0;
    m_roi_line.center_point.pose.orientation.y = 0.0;
    m_roi_line.center_point.pose.orientation.z = 0.0;
    m_roi_line.center_point.color.r = 0.0f;
    m_roi_line.center_point.color.g = 1.0f;
    m_roi_line.center_point.color.b = 1.0f;
    m_roi_line.center_point.color.a = 1.0;
    m_roi_line.center_point.lifetime = ros::Duration();

    m_roi_line.roi.ns = "roi";
    m_roi_line.roi.id = 0;
    m_roi_line.roi.type = visualization_msgs::Marker::CUBE;
    m_roi_line.roi.action = visualization_msgs::Marker::ADD;
    m_roi_line.roi.color.r = 1.0f;
    m_roi_line.roi.color.g = 0.0f;
    m_roi_line.roi.color.b = 1.0f;
    m_roi_line.roi.color.a = 0.3;
    m_roi_line.roi.lifetime = ros::Duration();
}

void Display::InputWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &input_waypoints_ptr)
{
    // line strip
    m_input_line.line_strip.header.frame_id = input_waypoints_ptr->header.frame_id;
    m_input_line.line_strip.header.stamp = ros::Time(0);

    
    m_input_line.line_strip.points.clear();
    for (auto p : input_waypoints_ptr->poses){
        m_input_line.line_strip.points.push_back(p.position);
    }

    // orientation
    m_input_line.orientation.header.frame_id = input_waypoints_ptr->header.frame_id;
    m_input_line.orientation.header.stamp = ros::Time(0);
    m_input_line.orientation.pose = input_waypoints_ptr->poses.back();
}

void Display::InputPositionCallback(const uav_msgs::CarState::ConstPtr &current_point_ptr)
{
    m_input_line.center_point.header.frame_id = current_point_ptr->header.frame_id;
    m_input_line.center_point.header.stamp = ros::Time(0);

    m_input_line.center_point.pose.position.x = current_point_ptr->pose.pose.position.x;
    m_input_line.center_point.pose.position.y = current_point_ptr->pose.pose.position.y;
    m_input_line.center_point.pose.position.z = current_point_ptr->pose.pose.position.z;
}

void Display::DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &desired_waypoints_ptr)
{
    m_desired_line.line_strip.header.frame_id = desired_waypoints_ptr->header.frame_id;
    m_desired_line.line_strip.header.stamp = ros::Time(0);

    m_desired_line.line_strip.points.clear();
    for (auto p : desired_waypoints_ptr->poses){
        m_desired_line.line_strip.points.push_back(p.position);
    }
}

void Display::ROIWaypointsCallback(const uav_msgs::PolyfitLane::ConstPtr &roi_waypoints_ptr)
{
    m_roi_line.line_strip.header.frame_id = roi_waypoints_ptr->header.frame_id;
    m_roi_line.line_strip.header.stamp = ros::Time(0);

    m_roi_line.line_strip.points.clear();
    for (auto p : roi_waypoints_ptr->points){
        m_roi_line.line_strip.points.push_back(p);
    }
}

void Display::ROICurrPositionCallback(const nav_msgs::Odometry::ConstPtr &current_point_ptr)
{
    m_roi_line.center_point.header.frame_id = current_point_ptr->header.frame_id;
    m_roi_line.center_point.header.stamp = ros::Time(0);

    m_roi_line.center_point.pose.position.x = current_point_ptr->pose.pose.position.x;
    m_roi_line.center_point.pose.position.y = current_point_ptr->pose.pose.position.y;
    m_roi_line.center_point.pose.position.z = current_point_ptr->pose.pose.position.z;
}

void Display::ROICallback(const uav_msgs::Roi::ConstPtr &roi_ptr)
{
    m_roi_line.roi.header.frame_id = roi_ptr->header.frame_id;
    m_roi_line.roi.header.stamp = ros::Time(0);

    m_roi_line.roi.pose.position.x = roi_ptr->pose.position.x;
    m_roi_line.roi.pose.position.y = roi_ptr->pose.position.y;
    m_roi_line.roi.pose.position.z = roi_ptr->pose.position.z;

    m_roi_line.roi.pose.orientation.w = roi_ptr->pose.orientation.w;
    m_roi_line.roi.pose.orientation.x = roi_ptr->pose.orientation.x;
    m_roi_line.roi.pose.orientation.y = roi_ptr->pose.orientation.y;
    m_roi_line.roi.pose.orientation.z = roi_ptr->pose.orientation.z;

    m_roi_line.roi.scale.x = roi_ptr->scale.x;
    m_roi_line.roi.scale.y = roi_ptr->scale.y;
    m_roi_line.roi.scale.z = roi_ptr->scale.z;
}

void Display::TimerCallback(const ros::TimerEvent& event)
{
    m_input_line.self.markers.clear();
    m_input_line.self.markers.push_back(m_input_line.line_strip);
    m_input_line.self.markers.push_back(m_input_line.orientation);
    m_input_line.self.markers.push_back(m_input_line.center_point);
    m_input_line_pub.publish(m_input_line.self);

    m_desired_line.self.markers.clear();
    m_desired_line.self.markers.push_back(m_desired_line.line_strip);
    m_desired_line_pub.publish(m_desired_line.self);    

    m_roi_line.self.markers.clear();
    m_roi_line.self.markers.push_back(m_roi_line.line_strip);
    m_roi_line.self.markers.push_back(m_roi_line.center_point);
    m_roi_line.self.markers.push_back(m_roi_line.roi);
    m_roi_line_pub.publish(m_roi_line.self);
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "display_node");
    Display display;
    // display.Run();

    ros::spin();

    return 0;
}