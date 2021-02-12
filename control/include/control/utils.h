#ifndef __UTILS_H__
#define __UTILS_H__
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geographic_msgs/GeoPoseStamped.h>

namespace control{

struct Euler {
    double r;
    double p;
    double y;
};

struct VehicleState{
    geometry_msgs::PoseStamped local_pose;
    geographic_msgs::GeoPoseStamped global_pose_raw;   // [deg/1e-7]
    geographic_msgs::GeoPoseStamped global_pose_deg;
    geographic_msgs::GeoPoseStamped global_pose_rad;
    geographic_msgs::GeoPoseStamped prev_global_pose_rad;
    int g_speed_raw;                                // [mm/s]
    double g_speed;                                 // [m/s]
    int heading_raw;                                // [deg/1e-5]
    double heading_deg;
    double heading_rad;
    double east;
    double north;
};

class Utils
{
public:
    Utils();
    ~Utils();

    Euler Quat2Euler(const geometry_msgs::Quaternion& quat_msg);
    float Distance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    float Degree2Rad(float degree);
    double MeridionalRadius(double a, double b, double lat);
    double NormalRadius(double a, double b, double lat);
};

}

#endif // __UTILS_H__