#include "control/tf_broadcaster.h"

TfBroadcaster::TfBroadcaster()
{

}

TfBroadcaster::~TfBroadcaster()
{}




void TfBroadcaster::CallBackNovatelINSPVA(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr)
{
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp =  ;
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = turtle_name;

	transformStamped.transform.translation.x = msg->x;
	transformStamped.transform.translation.y = msg->y;
	transformStamped.transform.translation.z = 0.0;
	
	tf2::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);


    static tf::TransformBroadcaster tf_novatel;

    ros::Time pres_time = msg->header.stamp;

    // Get quternion
    tf::Quaternion quternion;
    quternion.setRPY(msg->roll * M_PI / 180., msg->pitch * M_PI / 180., (-1*msg->azimuth + 90.0) * M_PI / 180.);

    // Get offset
    geometry_msgs::PoseStamped novatel_enu_pose = ConvertToMapFrame(msg->latitude, msg->longitude, msg->height);
    tf::Vector3 offset = tf::Vector3(novatel_enu_pose.pose.position.x, novatel_enu_pose.pose.position.y, novatel_enu_pose.pose.position.z);

    // brodcast
    tf_novatel.sendTransform( 
        tf::StampedTransform(tf::Transform(quternion, offset), pres_time, "map", "novatel"));

    tf::Vector3 offset_velodyne = tf::Vector3(0,0,0.3);
    tf::Quaternion quternion_velodyne;
    quternion_velodyne.setRPY(0,0,0);

    // brodcast
    tf_novatel.sendTransform( 
        tf::StampedTransform(tf::Transform(quternion_velodyne, offset_velodyne), pres_time, "novatel", "velodyne"));
}

geometry_msgs::PoseStamped TfBroadcaster::ConvertToMapFrame(float lat, float lon, float hgt)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  

    double m_dRefLatitude_deg = 37.3962732790;
    double m_dRefLongitude_deg = 127.1066872418;
    // double m_dRefLatitude_deg = init_gnss.latitude;
    // double m_dRefLongitude_deg = init_gnss.longitude;

    hgt = m_dMapHeight;

    dKappaLat = FnKappaLat( m_dRefLatitude_deg , hgt );
    dKappaLon = FnKappaLon( m_dRefLatitude_deg , hgt );

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Quaternion quat;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";

    pose.pose.position.x = (lon-m_dRefLongitude_deg)/dKappaLon;
    pose.pose.position.y = (lat-m_dRefLatitude_deg)/dKappaLat;
    pose.pose.position.z = hgt;

    return(pose);

}

double TfBroadcaster::FnKappaLat(double dRef_Latitude, double dHeight)
{
	double dKappaLat = 0;
	double Denominator = 0;
	double dM = 0;

	// estimate the meridional radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(m_utils.Degree2Rad(dRef_Latitude)), 2));
	dM = GEOD_A * (1 - GEOD_e2) / pow(Denominator, 3);

	// Curvature for the meridian
	dKappaLat = 1 / m_utlis.Rad2Degree(dM + dHeight);

	return dKappaLat;
}

double TfBroadcaster::FnKappaLon(double dRef_Latitude, double dHeight)
{
	double dKappaLon = 0;
	double Denominator = 0;
	double dN = 0;

	// estimate the normal radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(m_utils.Degree2Rad(dRef_Latitude)), 2));
	dN = GEOD_A / Denominator;

	// Curvature for the meridian
	dKappaLon = 1 / m_utils.Rad2Degree((dN + dHeight) * cos(m_utils.Degree2Rad(dRef_Latitude)));

	return dKappaLon;
}