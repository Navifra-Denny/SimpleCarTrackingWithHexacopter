#ifndef _VISUALIZEDETECTEDOBJECTS_HPP
#define _VISUALIZEDETECTEDOBJECTS_HPP

#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "uav_msgs/DetectedObject.h"
#include "uav_msgs/DetectedObjectArray.h"

#define __APP_NAME__ "visualize_perception"

class VisualizeDetectedObjects
{
private:
    // Node Handler
    ros::NodeHandle _node_handle;

    // Subscriber
    ros::Subscriber _subscriber_detected_objects;

    // Publisher
    ros::Publisher  _publisher_markers;

    // %%%%%%%%%% Param %%%%%%%%%%% 
    std::string ros_namespace_;   
    // Publisher topic name
    std::string _objects_markers_topic;     // no namespace pub topic name
    std::string markers_out_topic;      // Publish topic name
    // Subscriber topic name
    std::string _objects_topic;

    const double _label_height;
    const double _arrow_height;
    const double _object_max_linear_size;
    double _object_speed_threshold;
    double _arrow_speed_threshold;
    double _marker_display_duration;


    // Variable
    std::vector<double> color;
    std_msgs::ColorRGBA _label_color, _arrow_color, _hull_color, _box_color, _centroid_color;
    int _marker_id;

public:
    VisualizeDetectedObjects();
    ~VisualizeDetectedObjects();

private:
    void getParam();
    void DetectedObjectsCallback(const uav_msgs::DetectedObjectArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToLabels(const uav_msgs::DetectedObjectArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToArrows(const uav_msgs::DetectedObjectArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToHulls(const uav_msgs::DetectedObjectArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToBoxes(const uav_msgs::DetectedObjectArray &in_objects);
    visualization_msgs::MarkerArray ObjectsToCentroids(const uav_msgs::DetectedObjectArray &in_objects);


    // 
    bool IsObjectValid(const uav_msgs::DetectedObject &in_object);

    // Supplementary function 
    std::string ColorToString(const std_msgs::ColorRGBA &in_color);
    std_msgs::ColorRGBA ParseColor(const std::vector<double> &in_color);
    float CheckColor(double value); 
    float CheckAlpha(double value);

};






#endif