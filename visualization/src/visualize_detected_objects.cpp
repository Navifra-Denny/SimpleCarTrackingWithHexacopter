#include "visualize_detected_objects.hpp"

VisualizeDetectedObjects::VisualizeDetectedObjects() : _arrow_height(0.5),_label_height(1.0), _object_max_linear_size(50.0)
{
    // Get namespace 
    ros_namespace_ = ros::this_node::getNamespace();    // 현재: /detection
    if (ros_namespace_.substr(0, 2) == "//") ros_namespace_.erase(ros_namespace_.begin()); 

    // Get parameter 
    getParam();

    // Publish topic name
    markers_out_topic = ros_namespace_ + _objects_markers_topic;    // 현재 : /detection/objects_markers
    // Subscribe topic name
    _objects_topic = ros_namespace_ + _objects_topic;       // 현재 : /detection/objects

    // Subscribe Detected ObjectArray
    _subscriber_detected_objects = _node_handle.subscribe(_objects_topic, 1, &VisualizeDetectedObjects::DetectedObjectsCallback, this);

    // Publish MarkerArray
    _publisher_markers = _node_handle.advertise<visualization_msgs::MarkerArray>(markers_out_topic, 1);

}

VisualizeDetectedObjects::~VisualizeDetectedObjects()
{

}

void VisualizeDetectedObjects::getParam()
{
    _node_handle.getParam("visualization/objects_markers_topic", _objects_markers_topic);
    ROS_INFO("[%s] objects_markers_topic: %s", __APP_NAME__, _objects_markers_topic.c_str());
    
    _node_handle.getParam("visualization/objects_topic", _objects_topic);
    ROS_INFO("[%s] objects_topic: %s", __APP_NAME__, _objects_topic.c_str());

    _node_handle.getParam("visualization/object_speed_threshold", _object_speed_threshold);
    ROS_INFO("[%s] object_speed_threshold: %.2f", __APP_NAME__, _object_speed_threshold);

    _node_handle.getParam("visualization/arrow_speed_threshold", _arrow_speed_threshold);
    ROS_INFO("[%s] arrow_speed_threshold: %.2f", __APP_NAME__, _arrow_speed_threshold);

    _node_handle.getParam("visualization/marker_display_duration", _marker_display_duration);
    ROS_INFO("[%s] marker_display_duration: %.2f", __APP_NAME__, _marker_display_duration);

    // _node_handle.getParam("visualization/label_color", color);
    // _label_color = ParseColor(color);
    // ROS_INFO("[%s] label_color: %s", __APP_NAME__, ColorToString(_label_color).c_str());

    // _node_handle.getParam("visualization/arrow_color", color);
    // _arrow_color = ParseColor(color);
    // ROS_INFO("[%s] arrow_color: %s", __APP_NAME__, ColorToString(_arrow_color).c_str());

    // _node_handle.getParam("visualization/hull_color", color);
    // _hull_color = ParseColor(color);
    // ROS_INFO("[%s] hull_color: %s", __APP_NAME__, ColorToString(_hull_color).c_str());

    // _node_handle.getParam("visualization/box_color", color);
    // _box_color = ParseColor(color);
    // ROS_INFO("[%s] box_color: %s", __APP_NAME__, ColorToString(_box_color).c_str());

    // _node_handle.getParam("visualization/centroid_color", color);
    // _centroid_color = ParseColor(color);
    // ROS_INFO("[%s] centroid_color: %s", __APP_NAME__, ColorToString(_centroid_color).c_str());

}


void VisualizeDetectedObjects::DetectedObjectsCallback(const uav_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray label_markers, arrow_markers, polygon_hulls, bounding_boxes, centroid_markers;
    visualization_msgs::MarkerArray visualization_markers;

    label_markers = ObjectsToLabels(in_objects);
    arrow_markers = ObjectsToArrows(in_objects);
    polygon_hulls = ObjectsToHulls(in_objects);
    bounding_boxes = ObjectsToBoxes(in_objects);
    centroid_markers = ObjectsToCentroids(in_objects);

    visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                        label_markers.markers.begin(), label_markers.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        arrow_markers.markers.begin(), arrow_markers.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        polygon_hulls.markers.begin(), polygon_hulls.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(),
                                        bounding_boxes.markers.begin(), bounding_boxes.markers.end());
    visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                        centroid_markers.markers.begin(), centroid_markers.markers.end());


    _publisher_markers.publish(visualization_markers);
}

visualization_msgs::MarkerArray VisualizeDetectedObjects::ObjectsToLabels(const uav_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray label_markers;
    for (auto const &object: in_objects.objects)
    {
        if(IsObjectValid(object)) 
        {
            visualization_msgs::Marker label_marker;

            label_marker.lifetime = ros::Duration(_marker_display_duration);
            label_marker.header = in_objects.header;
            label_marker.ns = ros_namespace_ + "/label_markers";
            label_marker.action = visualization_msgs::Marker::ADD;
            label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label_marker.scale.x = 0.2;
            label_marker.scale.y = 0.2;
            label_marker.scale.z = 0.2;

            label_marker.color.r = object.color.r/255.0f;
            label_marker.color.g = object.color.g/255.0f;
            label_marker.color.b = object.color.b/255.0f;
            label_marker.color.a = 1.0;
            label_marker.id = object.id;
            // ROS_ERROR_STREAM(label_marker.color);

            if (!object.label.empty() && object.label != "unknown") label_marker.text = object.label + " "; //Object Class if available

            std::stringstream distance_stream;
            distance_stream << std::fixed << std::setprecision(1)
                            << sqrt((object.pose.position.x * object.pose.position.x) +
                                    (object.pose.position.y * object.pose.position.y));

            std::string distance_str = distance_stream.str() + " m";
            label_marker.text += distance_str;

            if (object.velocity_reliable)
            {
                double velocity = object.velocity.linear.x;
                if (velocity < -0.1) velocity *= -1;
                if (abs(velocity) < _object_speed_threshold) velocity = 0.0; 

                tf::Quaternion q(object.pose.orientation.x, object.pose.orientation.y,
                                object.pose.orientation.z, object.pose.orientation.w);

                double roll, pitch, yaw;
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                // convert m/s to km/h
                std::stringstream kmh_velocity_stream;
                kmh_velocity_stream << std::fixed << std::setprecision(1) << (velocity * 3.6);
                std::string text = "\n<" + std::to_string(object.id) + "> " + kmh_velocity_stream.str() + " km/h";
                label_marker.text += text;


            }
            label_marker.pose.position.x = object.pose.position.x;
            label_marker.pose.position.y = object.pose.position.y;
            label_marker.pose.position.z = _label_height;
            label_marker.scale.z = 1.0;
            if (!label_marker.text.empty()) label_markers.markers.push_back(label_marker);
        }
    }

    return label_markers;
}

visualization_msgs::MarkerArray VisualizeDetectedObjects::ObjectsToArrows(const uav_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray arrow_markers;
    
    for(auto const &object: in_objects.objects)
    {
        if(IsObjectValid(object) && object.pose_reliable)
        {
            double velocity = object.velocity.linear.x;

            if(abs(velocity) >= _arrow_speed_threshold)
            {
                visualization_msgs::Marker arrow_marker;
                arrow_marker.lifetime = ros::Duration(_marker_display_duration);

                tf::Quaternion q(object.pose.orientation.x,
                                 object.pose.orientation.y,
                                 object.pose.orientation.z,
                                 object.pose.orientation.w);

                double roll, pitch, yaw;

                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
                 if (velocity < -0.1)
                 {
                    yaw += M_PI;
                    // normalize angle
                    while (yaw > M_PI)
                        yaw -= 2. * M_PI;
                    while (yaw < -M_PI)
                        yaw += 2. * M_PI;
                 }

                tf::Matrix3x3 obs_mat; 
                tf::Quaternion q_tf;

                obs_mat.setEulerYPR(yaw, 0, 0);  // yaw, pitch, roll
                obs_mat.getRotation(q_tf);

                arrow_marker.header = in_objects.header;
                arrow_marker.ns = ros_namespace_ + "/arrow_markers";
                arrow_marker.action = visualization_msgs::Marker::ADD;
                arrow_marker.type = visualization_msgs::Marker::ARROW;

                arrow_marker.color.r = object.color.r/255.0f;
                arrow_marker.color.g = object.color.g/255.0f;
                arrow_marker.color.b = object.color.b/255.0f;
                arrow_marker.color.a = 1.0;

                arrow_marker.id = object.id;

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                arrow_marker.pose.position.x = object.pose.position.x;
                arrow_marker.pose.position.y = object.pose.position.y;
                arrow_marker.pose.position.z = _arrow_height;

                arrow_marker.pose.orientation.x = q_tf.getX();
                arrow_marker.pose.orientation.y = q_tf.getY();
                arrow_marker.pose.orientation.z = q_tf.getZ();
                arrow_marker.pose.orientation.w = q_tf.getW();

                // Set the scale of the arrow -- 1x1x1 here means 1m on a side
                arrow_marker.scale.x = 3;
                arrow_marker.scale.y = 0.1;
                arrow_marker.scale.z = 0.1;

                arrow_markers.markers.push_back(arrow_marker);
            }
        }
    }
    return arrow_markers;
}

visualization_msgs::MarkerArray VisualizeDetectedObjects::ObjectsToHulls(const uav_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray polygon_hulls;

    for(auto const &object: in_objects.objects)
    {
        if (IsObjectValid(object) && !object.convex_hull.polygon.points.empty() && object.label == "unknown")
        {
            visualization_msgs::Marker hull;
            hull.lifetime = ros::Duration(_marker_display_duration);
            hull.header = in_objects.header;
            hull.type = visualization_msgs::Marker::LINE_STRIP;
            hull.action = visualization_msgs::Marker::ADD;
            hull.ns = ros_namespace_ + "/hull_markers";
            hull.id = object.id;
            hull.scale.x = 0.1;

            for (auto const &point: object.convex_hull.polygon.points)
            {
                geometry_msgs::Point tmp_point;
                tmp_point.x = point.x;
                tmp_point.y = point.y;
                tmp_point.z = point.z;
                hull.points.push_back(tmp_point);
            }

            // hull.color = object.color;
            hull.color.r = object.color.r/255.0f;
            hull.color.g = object.color.g/255.0f;
            hull.color.b = object.color.b/255.0f;
            hull.color.a = 1.0;

            polygon_hulls.markers.push_back(hull);
        }
    }

    return polygon_hulls;
}


visualization_msgs::MarkerArray VisualizeDetectedObjects::ObjectsToBoxes(const uav_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray object_boxes;

    for (auto const &object: in_objects.objects)
    {
        if(IsObjectValid(object) &&     // 조건 tracking 때 추가해야함 -> 일단 detection 단계에서 뽑기위해 이렇게 설정 
          (object.dimensions.x + object.dimensions.y + object.dimensions.z) < _object_max_linear_size)
          {
                visualization_msgs::Marker box;

                box.lifetime = ros::Duration(_marker_display_duration);
                box.header = in_objects.header;
                box.type = visualization_msgs::Marker::CUBE;
                box.action = visualization_msgs::Marker::ADD;
                box.ns = ros_namespace_ + "/box_markers";
                box.id = object.id;
                box.scale = object.dimensions;
                box.pose.position = object.pose.position;

                // if (object.pose_reliable) box.pose.orientation = object.pose.orientation;
                box.pose.orientation = object.pose.orientation;

                box.color.r = object.color.r/255.0f;
                box.color.g = object.color.g/255.0f;
                box.color.b = object.color.b/255.0f;
                box.color.a = 0.5;

                object_boxes.markers.push_back(box);
          }
    }
    
    return object_boxes;
}

visualization_msgs::MarkerArray VisualizeDetectedObjects::ObjectsToCentroids(const uav_msgs::DetectedObjectArray &in_objects)
{
    visualization_msgs::MarkerArray centroid_markers;

    for (auto const &object: in_objects.objects)
    {
        if(IsObjectValid(object))
        {
            visualization_msgs::Marker centroid_marker;
            centroid_marker.lifetime = ros::Duration(_marker_display_duration);
            centroid_marker.header = in_objects.header;
            centroid_marker.type = visualization_msgs::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::Marker::ADD;
            centroid_marker.pose = object.pose;
            centroid_marker.ns = ros_namespace_ + "/centroid_markers";

            centroid_marker.scale.x = 0.2;
            centroid_marker.scale.y = 0.2;
            centroid_marker.scale.z = 0.2;

            // centroid_marker.color.r = object.color.r/255.0f;
            // centroid_marker.color.g = object.color.g/255.0f;
            // centroid_marker.color.b = object.color.b/255.0f;

            centroid_marker.color.r = 0.0f;
            centroid_marker.color.g = 0.0f;
            centroid_marker.color.b = 0.0f;
            centroid_marker.color.a = 1.0;
            centroid_marker.id = object.id;
            
            centroid_markers.markers.push_back(centroid_marker);
        }
    }

    return centroid_markers;
}




bool VisualizeDetectedObjects::IsObjectValid(const uav_msgs::DetectedObject &in_object)
{   
    if(!in_object.valid || 
      std::isnan(in_object.pose.orientation.x) ||
      std::isnan(in_object.pose.orientation.y) ||
      std::isnan(in_object.pose.orientation.z) ||
      std::isnan(in_object.pose.orientation.w) ||
      std::isnan(in_object.pose.position.x) ||
      std::isnan(in_object.pose.position.y) ||
      std::isnan(in_object.pose.position.z) ||
      (in_object.pose.position.x == 0.) ||
      (in_object.pose.position.y == 0.) ||
      (in_object.dimensions.x <= 0.) ||
      (in_object.dimensions.y <= 0.) ||
      (in_object.dimensions.z <= 0.)
    )
    {
        return false;
    }    

    return true;
}


// Supplementary function 
std::string VisualizeDetectedObjects::ColorToString(const std_msgs::ColorRGBA &in_color)
{
  std::stringstream stream;

  stream << "{R:" << std::fixed << std::setprecision(1) << in_color.r*255 << ", ";
  stream << "G:" << std::fixed << std::setprecision(1) << in_color.g*255 << ", ";
  stream << "B:" << std::fixed << std::setprecision(1) << in_color.b*255 << ", ";
  stream << "A:" << std::fixed << std::setprecision(1) << in_color.a << "}";
  return stream.str();
}

std_msgs::ColorRGBA VisualizeDetectedObjects::ParseColor(const std::vector<double> &in_color)
{
  std_msgs::ColorRGBA color;
  float r,g,b,a;
  if (in_color.size() == 4) //r,g,b,a
  {
    color.r = CheckColor(in_color[0]);
    color.g = CheckColor(in_color[1]);
    color.b = CheckColor(in_color[2]);
    color.a = CheckAlpha(in_color[3]);
  }
  return color;
}

float VisualizeDetectedObjects::CheckColor(double value)
{
  float final_value;
  if (value > 255.)
    final_value = 1.f;
  else if (value < 0)
    final_value = 0.f;
  else
    final_value = value/255.f;
  return final_value;
}

float VisualizeDetectedObjects::CheckAlpha(double value)
{
  float final_value;
  if (value > 1.)
    final_value = 1.f;
  else if (value < 0.1)
    final_value = 0.1f;
  else
    final_value = value;
  return final_value;
}