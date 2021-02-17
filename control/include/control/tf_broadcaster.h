#ifndef __TF_BROADCASTER_H__
#define __TF_BROADCASTER_H__

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/PoseStamped.h>
#include <novatel_oem7_msgs/INSPVA.h>

#include "control/utils.h"

class TfBroadcaster{
public:
    TfBroadcaster();
    ~TfBroadcaster();

    const double GEOD_A = 6378137.0;//SemiMajorAxis
    const double GEOD_e2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2

private:
	ros::NodeHandle m_nh;
    control::Utils m_utils;

private:
    void CallBackNovatelINSPVA(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr);

    double m_map_height_m;
    double m_ref_latitude_deg;
    double m_ref_longitude_deg;

    // param

    geometry_msgs::PoseStamped ConvertToMapFrame(float lat, float lon, float hgt);
    double FnKappaLat(double dRef_Latitude, double dHeight);
    double FnKappaLon(double dRef_Latitude, double dHeight);


};





#endif // __TF_BROADCASTER_H__