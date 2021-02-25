#ifndef __KF_HPP__
#define __KF_HPP__

#include "Eigen/Dense"
#include <ros/ros.h>
#include <vector>
#include <string>

#include "uav_msgs/DetectedObject.h"


enum TrackingState : int
{
    Die = 0,
    Init = 1,
    Stable = 4,
    Occlusion = 5,
    Lost = 15
};

class Vector
{
public:
    double x_;
    double y_;
public:
    Vector()=default;
    Vector(double x, double y){
        x_ = x;
        y_ = y;
    }
    ~Vector()
    {}
};

class KF
{
public:
    // imm 참고한 부분
    int kf_id_;
    int num_state_;
    int num_measure_state_;

    int tracking_num_;
    long long time_;


    int lifetime_;
    
    //object msg information
    bool is_static_;
    bool is_stable_;
    uav_msgs::DetectedObject object_;
    std::string label_;

    std_msgs::ColorRGBA color_;
    std_msgs::Header header_;
    
    // for env classification
    double vel_;
    double yaw_;
    std::vector<double> vel_history_;

    // estimate heading
    Vector reference_vec_;
    Vector ego_vec_;

    // LiDAR measurement noise standard deviation in [m]
    double std_lidar_px_;
    double std_lidar_py_;


    // state vector : [pos_x, pos_y, vel_x, vel_y]
    Eigen::MatrixXd state_pre_;
    Eigen::MatrixXd state_post_;

    // measurement vector
    Eigen::VectorXd init_meas_;
    Eigen::MatrixXd measure_pre_;

    // covariance matrix
    Eigen::MatrixXd error_cov_pre_;
    Eigen::MatrixXd error_cov_post_;

    // Q, R covariance matrix
    Eigen::MatrixXd process_noise_cov_;
    Eigen::MatrixXd measure_noise_cov_;

    // transition matrix
    Eigen::MatrixXd transition_mat_;
    // measurement matrix
    Eigen::MatrixXd measure_mat_;

    Eigen::MatrixXd kalman_gain_;


    // function
    KF();
    virtual ~KF();

    void initialize(const Eigen::VectorXd& z, const double timestamp, const int target_id, const std_msgs::ColorRGBA color);
    void prediction(const double dt);
    void predictionMotion(const double dt);
    void predictionLidarMeasurement();
    void update(const std::vector<uav_msgs::DetectedObject>& object_vec);
    void updateKalmanGain();
    void updateMotion(const std::vector<uav_msgs::DetectedObject>& object_vec);

    // helper for calculating heading 
    Vector VectorSubtractVector(Vector dest, Vector start);
    double GetHeadingAngle(Vector ref, Vector ego_vec);
};








#endif