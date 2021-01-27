#include "ros/ros.h"
#include <ros/spinner.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

class Display
{
public:
    Display();
    virtual ~Display();

    void InputWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array);
    void DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array);

private:
    // Node Handler
	ros::NodeHandle m_nh;

	// subscriber
	ros::Subscriber m_input_waypoints_sub;
	ros::Subscriber m_desired_waypoints_sub;

    // publisher
    ros::Publisher m_input_line_pub;
    ros::Publisher m_desired_line_pub;

private:
    void GenerateDesiredWaypoints();
};

Display::Display()
{
    // subscriber init
	m_input_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>("/generate_waypoints_node/input_waypoints", 1, boost::bind(&Display::InputWaypointsCallback, this, _1));
	m_desired_waypoints_sub = m_nh.subscribe<geometry_msgs::PoseArray>("/generate_waypoints_node/desired_waypoints", 1, boost::bind(&Display::DesiredWaypointsCallback, this, _1));

    // publisher init
    // m_input_line_pub = m_nh.advertise<visualization_msgs::Marker> ("/Marker/input_line", 1);
    m_input_line_pub = m_nh.advertise<visualization_msgs::MarkerArray> ("/Marker/input_line", 1);
    m_desired_line_pub = m_nh.advertise<visualization_msgs::MarkerArray> ("/Marker/desired_line", 1);
}

Display::~Display()
{}

void Display::InputWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array)
{
    auto poses = pose_array->poses;
    visualization_msgs::MarkerArray input_line;

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/world_ned";
    line_strip.header.stamp = ros::Time::now();

    line_strip.ns = "line";
    line_strip.id = 0;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;

    for (auto p : poses){
        line_strip.points.push_back(p.position);
    }

    line_strip.scale.x = 0.05;
    line_strip.scale.y = 0.05;
    line_strip.scale.z = 0.05;

    line_strip.color.r = 0.0f;
    line_strip.color.g = 1.0f;
    line_strip.color.b = 0.0f;
    line_strip.color.a = 1.0;

    line_strip.lifetime = ros::Duration();
    input_line.markers.push_back(line_strip);

    visualization_msgs::Marker orientation;
    orientation.header.frame_id = "/world_ned";
    orientation.header.stamp = ros::Time::now();

    orientation.ns = "arrow";
    orientation.id = 0;

    orientation.type = visualization_msgs::Marker::ARROW;
    orientation.action = visualization_msgs::Marker::ADD;

    orientation.pose = poses.back();

    orientation.scale.x = 0.5;
    orientation.scale.y = 0.07;
    orientation.scale.z = 0.07;

    orientation.color.r = 1.0f;
    orientation.color.g = 0.0f;
    orientation.color.b = 0.0f;
    orientation.color.a = 1.0;

    orientation.lifetime = ros::Duration();
    input_line.markers.push_back(orientation);

    m_input_line_pub.publish(input_line);
}

void Display::DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr &pose_array)
{
    auto poses = pose_array->poses;
    visualization_msgs::MarkerArray desired_line;

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/world_ned";
    line_strip.header.stamp = ros::Time::now();

    line_strip.ns = "line";
    line_strip.id = 0;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;

    for (auto p : poses){
        line_strip.points.push_back(p.position);
    }

    line_strip.scale.x = 0.05;
    line_strip.scale.y = 0.05;
    line_strip.scale.z = 0.05;

    line_strip.color.r = 1.0f;
    line_strip.color.g = 0.0f;
    line_strip.color.b = 0.0f;
    line_strip.color.a = 1.0;

    line_strip.lifetime = ros::Duration();
    desired_line.markers.push_back(line_strip);

    m_desired_line_pub.publish(desired_line);
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