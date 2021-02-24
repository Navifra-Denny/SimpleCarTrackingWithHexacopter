#include "control/utils.h"
#include "math.h"
#include <iomanip>
#include <sstream>

namespace control
{

Utils::Utils() :
    GEOD_A(6378137.0),
    GEOD_e2(0.00669437999014),
    EARTH_RADIUS_M(6371.0)
{}

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

float Utils::Distance2D(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto distance_m = Size(delta_x, delta_y);

    return distance_m;
}

float Utils::Distance3D(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto delta_z = point1.z - point2.z;
    auto distance_m = Size(delta_x, delta_y, delta_z);

    return distance_m;
}

// ref http://www.movable-type.co.uk/scripts/latlong.html
double Utils::DistanceFromLatLonInKm(geographic_msgs::GeoPoint point1, geographic_msgs::GeoPoint point2)
{
    double lat1 = point1.latitude;
    double lon1 = point1.longitude;

    double lat2 = point2.latitude;
    double lon2 = point2.longitude;
    
    double dLat = Deg2Rad(lat2 - lat1);
    double dLon = Deg2Rad(lon2 - lon1);

    double a = 
        sin(dLat/2) * sin(dLat/2) + 
        cos(Deg2Rad(lat1)) * cos(Deg2Rad(lat2)) * 
        sin(dLon/2) * sin(dLon/2);

    double c = 2*atan2(sqrt(a), sqrt(1 - a)); 
    double distance_m = EARTH_RADIUS_M * c; // Distance in km
    return distance_m;
}

double Utils::Deg2Rad(double degree)
{
    double rad = degree * (M_PI / 180.0);
	return rad; 
}

double Utils::Rad2Deg(double rad)
{
	double degree = rad * (180.0 / M_PI);
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

bool Utils::IsNan(geographic_msgs::GeoPoint point)
{
    if (__isnan(point.latitude) || __isnan(point.longitude) || __isnan(point.altitude)){
        return true;
    }
    return false;
}

geometry_msgs::PoseStamped Utils::ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  

    dKappaLat = FnKappaLat( home_position.latitude , hgt );
    dKappaLon = FnKappaLon( home_position.latitude , hgt );

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = (lon-home_position.longitude)/dKappaLon;
    pose.pose.position.y = (lat-home_position.latitude)/dKappaLat;
    pose.pose.position.z = hgt;

    return(pose);
}

double Utils::FnKappaLat(double dRef_Latitude, double dHeight)
{
	double dKappaLat = 0;
	double Denominator = 0;
	double dM = 0;

	// estimate the meridional radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(Deg2Rad(dRef_Latitude)), 2));
	dM = GEOD_A * (1 - GEOD_e2) / pow(Denominator, 3);

	// Curvature for the meridian
	dKappaLat = Rad2Deg(1 / (dM + dHeight));

	return dKappaLat;
}

double Utils::FnKappaLon(double dRef_Latitude, double dHeight)
{
	double dKappaLon = 0;
	double Denominator = 0;
	double dN = 0;

	// estimate the normal radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(Deg2Rad(dRef_Latitude)), 2));
	dN = GEOD_A / Denominator;

	// Curvature for the meridian
	dKappaLon = Rad2Deg(1 / ((dN + dHeight) * cos(Deg2Rad(dRef_Latitude))));

	return dKappaLon;
}

double Utils::Size(double x, double y)
{
    double size = sqrt(pow(x, 2.0) + pow(y, 2.0));
    return size;
}

double Utils::Size(double x, double y, double z)
{
    double size = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));
    return size;
}

double Utils::ms_to_kmh(double ms)
{
    // 1s -> 1m
    // 3600s -> 3600m
    // 1h -> 3.6km
    double kmh = ms * 3.6;
    return kmh;
}

double Utils::VelNomalize(double value)
{
    const double MAX = 20.0;
    const double MIN = 0.1;

    if (value < MIN) value = MIN;
    else if (value > MAX) value = MAX;

    double ratio = value/(MAX - MIN);
    return ratio;
}

std::string Utils::ToString(double value)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    std::string s = stream.str();

    return s;
}

Eigen::Vector3d Utils::BodyRads2EnuRads(control::Euler euler, Eigen::Vector3d body_rads)
{
    auto roll = euler.r;
    auto pitch = euler.p;
    auto yaw = euler.y;
    
    double a00 = 1.0;   double a01 = sin(roll)*tan(pitch);      double a02 = cos(roll)*tan(pitch);
    double a10 = 0.0;   double a11 = cos(roll);                 double a12 = -sin(roll);
    double a20 = 0.0;   double a21 = sin(roll)*(1/cos(pitch));  double a22 = cos(roll)*(1/cos(pitch));
    
    Eigen::Matrix3d rotation;
    rotation << a00, a01, a02, a10, a11, a12, a20, a21, a22;

    Eigen::Vector3d enu_rads = rotation*body_rads;
    return enu_rads;
}

tf2::Quaternion Utils::Rads2Rad(Eigen::Vector3d rads, double dt)
{
    Eigen::Vector3d rad = rads*dt;
    
    tf2::Quaternion q;
    q.setRPY(rad(0), rad(1), rad(2));
    return q;
}
}