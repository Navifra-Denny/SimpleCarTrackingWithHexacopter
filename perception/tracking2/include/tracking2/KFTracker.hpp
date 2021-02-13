#ifndef __KFTRACKER_HPP__
#define __KFTRACKER_HPP__

#include <vector>

#include <ros/ros.h>

#include "uav_msgs/DetectedObject.h"
#include "uav_msgs/DetectedObjectArray.h"
#include "tracking2/KF.hpp"

class KFTracker
{
private:
    int target_id_;
    bool init_;
    double timestamp_;

    std::vector<KF> targets_;

    // paramteter
    double gating_threshold_;

    ros::NodeHandle node_handle_;
    ros::Subscriber sub_detected_array_;
    
    std_msgs::Header input_header_;


    void GetParam();
    void callback(const uav_msgs::DetectedObjectArray& input);
    void tracker(const uav_msgs::DetectedObjectArray& input, 
                uav_msgs::DetectedObjectArray& detected_objects_output);
    void initTracker(const uav_msgs::DetectedObjectArray& input, double timestamp);
    bool dataAssociation(const uav_msgs::DetectedObjectArray& input, 
                        const double dt, std::vector<bool>& matching_vec, 
                        std::vector<uav_msgs::DetectedObject>& object_vec, 
                        KF& target);

    void measurementValidation(const uav_msgs::DetectedObjectArray& input, KF& target,
                               const Eigen::VectorXd& measure_pre, 
                               std::vector<uav_msgs::DetectedObject>& object_vec,
                               std::vector<bool>& matching_vec);
    
    void secondInit(KF& target, const std::vector<uav_msgs::DetectedObject>& object_vec, double dt);
    void updateTargetWithAssociatedObject(const std::vector<uav_msgs::DetectedObject>& object_vec, KF& target);
    void updateTrackingNum(const std::vector<uav_msgs::DetectedObject>& object_vec, KF& target);
    void makeNewTargets(const double timestamp, const uav_msgs::DetectedObjectArray& input, 
                        const std::vector<bool>& matching_vec);



public:
    KFTracker();
    virtual ~KFTracker();
    void run();


};


#endif