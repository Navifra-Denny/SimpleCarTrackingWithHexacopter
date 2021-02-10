#include "control/utils.h"

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
    auto distance_m = std::sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);

    return distance_m;
}

float Utils::Degree2Rad(float degree)
{
	float rad = degree * M_PI/180;
	return rad; 
}
}