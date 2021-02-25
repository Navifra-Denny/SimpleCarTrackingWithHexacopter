#include "ros/ros.h"
#include <ros/spinner.h>

#include "uav_msgs/DetectionTime.h"
#include "uav_msgs/TrackingTime.h"
#include "uav_msgs/PerceptionTimer.h"

class CheckTimer{
public:
    CheckTimer();
    ~CheckTimer();

private:
    ros::NodeHandle m_nh;

    // Initialize subscriber
    ros::Subscriber m_tracking_time_sub;
    ros::Subscriber m_detection_time_sub;

    // Initialize publisher
    ros::Publisher m_perception_time_pub;

    // Initialize timer
    ros::Timer m_checktimer_timer;

    uav_msgs::PerceptionTimer m_perception_timer;
    
    void InitROS();

    void CheckTimerCallback(const ros::TimerEvent& event);
    
    void DetectionTimeCallback(const uav_msgs::DetectionTime::ConstPtr &time_ptr);
    void TrackingTimeCallback(const uav_msgs::TrackingTime::ConstPtr &time_ptr);
};

CheckTimer::CheckTimer()
{
    InitROS();
}

CheckTimer::~CheckTimer()
{}

void CheckTimer::InitROS()
{
    // Initialize subscriber
    // pub_timer_ = node_handle_.advertise<uav_msgs::TrackingTime>("tracking_timer", 1);
    // _pub_timer = nh.advertise<uav_msgs::DetectionTime>(ros_namespace_ + "/detection_timer",1);


    m_tracking_time_sub = m_nh.subscribe<uav_msgs::DetectionTime>("/detection/detection_timer", 10, boost::bind(&CheckTimer::DetectionTimeCallback, this, _1));
    m_detection_time_sub = m_nh.subscribe<uav_msgs::TrackingTime>("/tracking/tracking_timer", 10, boost::bind(&CheckTimer::TrackingTimeCallback, this, _1));

    // Initialize publisher
    m_perception_time_pub = m_nh.advertise<uav_msgs::PerceptionTimer>("/perception/check_timer_node/time", 10);

    // Initialize timer
    m_checktimer_timer = m_nh.createTimer(ros::Duration(0.1), &CheckTimer::CheckTimerCallback, this);
}


void CheckTimer::CheckTimerCallback(const ros::TimerEvent& event)
{
    m_perception_time_pub.publish(m_perception_timer);
}


void CheckTimer::DetectionTimeCallback(const uav_msgs::DetectionTime::ConstPtr &time_ptr)
{
    m_perception_timer.detection_time.transform = time_ptr->transform;
    m_perception_timer.detection_time.remove_ground = time_ptr->remove_ground;
    m_perception_timer.detection_time.threshold = time_ptr->threshold;
    m_perception_timer.detection_time.downsample = time_ptr->downsample;
    m_perception_timer.detection_time.gen_output = time_ptr->gen_output;
    m_perception_timer.detection_time.normal_segmentation = time_ptr->normal_segmentation;
    m_perception_timer.detection_time.cluster_and_color = time_ptr->cluster_and_color;
    m_perception_timer.detection_time.cluster_checking = time_ptr->cluster_checking;
    m_perception_timer.detection_time.cluster_publish = time_ptr->cluster_publish;
}

void CheckTimer::TrackingTimeCallback(const uav_msgs::TrackingTime::ConstPtr &time_ptr)
{
    m_perception_timer.tracking_time.tracking_init = time_ptr->tracking_init;
    m_perception_timer.tracking_time.gen_target = time_ptr->gen_target;
    m_perception_timer.tracking_time.static_classification = time_ptr->static_classification;
    m_perception_timer.tracking_time.gen_target_state = time_ptr->gen_target_state;
    m_perception_timer.tracking_time.gen_output = time_ptr->gen_output;
    m_perception_timer.tracking_time.remove_unnecessary_target = time_ptr->remove_unnecessary_target;
}


int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "check_timer_node");
    CheckTimer check_timer;

    ros::spin();
    return 0;
}