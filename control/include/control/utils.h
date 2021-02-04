#ifndef __UTILS_H__
#define __UTILS_H__
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace control{

struct Euler {
    double r;
    double p;
    double y;
};


class Utils
{
public:
    Utils();
    ~Utils();

    Euler Quat2Euler(const geometry_msgs::Quaternion& quat_msg);
    float Distance(geometry_msgs::Point point1, geometry_msgs::Point point2);
};

}

#endif // __UTILS_H__