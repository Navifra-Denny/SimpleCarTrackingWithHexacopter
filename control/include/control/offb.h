#ifndef __OFFB_H__
#define __OFFB_H__


#include <ros/ros.h>

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "control/utils.h"
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>

typedef common_utils::Utils Utils;
using namespace msr::airlib;


class Offboard
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    control::Utils m_utils;


    msr::airlib::MultirotorRpcLibClient m_client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

public:
    Offboard();
    virtual ~Offboard();

private:
    // subscriber

    // service client
    ros::Subscriber m_desired_waypoints_sub;

    // param
    // std::string m_vehicle_name_param;
    // float m_x_offset_m_param;
    // float m_z_offset_m_param;

private: // function
    void SetParam();
    
    void DesiredWaypointsCallback(const geometry_msgs::PoseArray::ConstPtr pose_array);

};

#endif //  __OFFB_H__