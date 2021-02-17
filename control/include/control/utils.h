#ifndef __UTILS_H__
#define __UTILS_H__
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPoint.h>

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

private:
    // const variable
    const double GEOD_A;
    const double GEOD_e2;
    const double EARTH_RADIUS_M;


public:
    Euler Quat2Euler(const geometry_msgs::Quaternion& quat_msg);
    float Distance(geometry_msgs::Point point1, geometry_msgs::Point point2);
    // ref http://www.movable-type.co.uk/scripts/latlong.html
    double DistanceFromLatLonInKm(geographic_msgs::GeoPoint point1, geographic_msgs::GeoPoint point2);
    double Deg2Rad(double degree);
    double Rad2Deg(double rad);
    double MeridionalRadius(double a, double b, double lat);
    double NormalRadius(double a, double b, double lat);
    bool IsNan(geometry_msgs::Point point);
    bool IsNan(geographic_msgs::GeoPoint point);
    geometry_msgs::PoseStamped ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position);
    double FnKappaLat(double dRef_Latitude, double dHeight);
    double FnKappaLon(double dRef_Latitude, double dHeight);
};
}

#endif // __UTILS_H__