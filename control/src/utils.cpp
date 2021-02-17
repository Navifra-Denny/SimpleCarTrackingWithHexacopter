#include "control/utils.h"
#include "math.h"

namespace control
{

Utils::Utils() {}
Utils::~Utils() {}

Euler Utils::Quat2Euler(const geometry_msgs::Quaternion& quat_msg)
{
	tf2::Quaternion quat_tf;
	double roll, pitch, yaw;
	tf2::fromMsg(quat_msg, quat_tf);
	tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
	
	Euler euler = {roll, pitch, yaw};

	return euler;
}


float Utils::Distance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto delta_z = point1.z - point2.z;
    auto distance_m = std::sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0) + pow(delta_z, 2.0));

    return distance_m;
}

double Utils::Degree2Rad(double degree)
{
	double rad = degree * M_PI/180;
	return rad; 
}

double Utils::Rad2Degree(double rad)
{
	double degree = rad * 180/M_PI;
	return degree;
}

double Utils::MeridionalRadius(double a, double b, double lat){
    return pow(a*b, 2) / sqrt( pow((pow( a*cos(lat), 2) + pow( b*sin(lat), 2 )), 3));
}

double Utils::NormalRadius(double a, double b, double lat){
    return (a*a) / sqrt(pow( a*cos(lat), 2 ) + pow( b*sin(lat), 2));
}

bool Utils::IsNan(geometry_msgs::Point point)
{
    if (__isnan(point.x) || __isnan(point.y) || __isnan(point.z)){
        return true;
    }
    return false;
}
}