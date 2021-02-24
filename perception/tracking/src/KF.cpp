#include "tracking/KF.hpp"

KF::KF()
: num_state_(4)
, num_measure_state_(2)
{
    // transition matrix 
    transition_mat_ = Eigen::MatrixXd(num_state_, num_state_);
    // measurement matrix 
    measure_mat_ = Eigen::MatrixXd(num_measure_state_, num_state_); 

    // initial state vector
    state_pre_ = Eigen::MatrixXd(num_state_, 1);     // x_cv_
    state_post_ = Eigen::MatrixXd(num_state_, 1);

    // initial measurement prediction
    init_meas_ = Eigen::VectorXd(num_measure_state_);   // second init에 필요 
    measure_pre_ = Eigen::MatrixXd(num_measure_state_, 1);   // H * x_k

    // initial covariance matrix
    error_cov_pre_ = Eigen::MatrixXd(num_state_, num_state_);
    error_cov_post_ = Eigen::MatrixXd(num_state_, num_state_);

    // initial Q, R covariance matrix
    process_noise_cov_ = Eigen::MatrixXd(num_state_, num_state_);    // q_cv_
    measure_noise_cov_ = Eigen::MatrixXd(num_measure_state_, num_measure_state_);   // r_cv


    // LiDAR measurement noise standard deviation in m
    std_lidar_px_ = 0.015;    // [m]
    std_lidar_py_ = 0.015;    // [m]

    lifetime_ = 0;

}

KF::~KF()
{

}

void KF::initialize(const Eigen::VectorXd& z, const double timestamp, const int target_id, const std_msgs::ColorRGBA color)
{
    kf_id_ = target_id;
    
    // init timestamp
    time_ = timestamp;

    color_ = color;

    // Measurement model
    measure_mat_ << 1, 0, 0, 0,
                    0, 1, 0, 0;

    // first measurement
    state_post_(0) = z(0);
    state_post_(1) = z(1);
    state_post_(2) = 0;
    state_post_(3) = 0;

    // init covariance matrix 
    error_cov_post_ << 0.01, 0, 0, 0,
                       0, 0.01, 0, 0,
                       0, 0, 3., 0,
                       0, 0, 0, 3.;
    
    // save initial measure for velocity calculation
    init_meas_ << z(0), z(1);
    std::cout << "init_meas_" <<init_meas_ << std::endl;

    state_pre_ = state_post_;
    error_cov_pre_ = error_cov_post_;

    // initialize Q, R covariance 
    process_noise_cov_ << 1., 0, 0, 0,
                          0, 1., 0, 0,
                          0, 0, 1.5, 0,
                          0, 0, 0, 1.5;

    measure_noise_cov_ << std_lidar_px_  * std_lidar_px_, 0, 0, std_lidar_py_ * std_lidar_py_; 


    // init tracking num 
    tracking_num_ = 1;
}

void KF::prediction(const double dt)
{
    // Prediction Motion model
    predictionMotion(dt);
    
    // Prediction Measurement
    predictionLidarMeasurement();
}

void KF::predictionMotion(const double dt)
{
    // std::cout << "예측 전 : " << state_pre_ << std::endl;
    // ROS_WARN_STREAM("-------------------------");

    // constant velocity 
    transition_mat_ << 1, 0, dt, 0,
                       0, 1, 0, dt,
                       0, 0, 1, 0,
                       0, 0, 0, 1;

    // estimate state 
    state_pre_ = transition_mat_ * state_post_;

    // std::cout << "state_pre : " << state_pre_ << std::endl;
    // estimate covariance 
    error_cov_pre_ = transition_mat_ * error_cov_post_ * transition_mat_.transpose() + process_noise_cov_;

    // std::cout << "error_cov_pre : " << error_cov_pre_ << std::endl;
}


void KF::predictionLidarMeasurement()
{
    measure_pre_ = measure_mat_ * state_pre_; 
}


void KF::update(const std::vector<uav_msgs::DetectedObject>& object_vec)
{
    // update kalman gain 
    updateKalmanGain();

    // update state variable x and state covariance p
    updateMotion(object_vec);

}

void KF::updateKalmanGain()
{
    kalman_gain_ = error_cov_pre_ * measure_mat_.transpose() * (measure_mat_ * error_cov_pre_ * measure_mat_.transpose() + measure_noise_cov_).inverse();
    std::cout << "kalman gain" <<kalman_gain_ << std::endl;
}

void KF::updateMotion(const std::vector<uav_msgs::DetectedObject>& object_vec)
{
    double num_meas = object_vec.size();

    Eigen::VectorXd measure_diff;
    measure_diff.setZero(num_measure_state_);

    ROS_WARN_STREAM(state_pre_(0));
    ROS_WARN_STREAM(state_pre_(1));
    ROS_WARN_STREAM(state_pre_(2));
    ROS_WARN_STREAM(state_pre_(3));


    for (size_t i = 0; i < num_meas; i++)
    {
        Eigen::VectorXd meas = Eigen::VectorXd(num_measure_state_);
        meas(0) = object_vec[i].pose.position.x;
        meas(1) = object_vec[i].pose.position.y;
        ROS_ERROR_STREAM(meas(0));
        ROS_ERROR_STREAM(meas(1));
        measure_diff = meas - measure_pre_;
    }

    // correct 
    state_post_ = state_pre_ + kalman_gain_ * measure_diff;
    ROS_INFO_STREAM(state_post_(0));
    ROS_INFO_STREAM(state_post_(1));
    ROS_INFO_STREAM(state_post_(2));
    ROS_INFO_STREAM(state_post_(3));

    error_cov_post_ = error_cov_pre_ - kalman_gain_ * measure_mat_ * error_cov_pre_;

}

