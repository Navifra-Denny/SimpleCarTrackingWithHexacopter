#ifndef __KFTRACKER_HPP__
#define __KFTRACKER_HPP__

#include <vector>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "uav_msgs/DetectedObject.h"
#include "uav_msgs/DetectedObjectArray.h"
#include "tracking/KF.hpp"

class KFTracker
{
private:
    int target_id_;
    bool init_;
    double timestamp_;

    std_msgs::ColorRGBA color;

    std::vector<KF> targets_;

    int life_time_threshold_;
    
    // static classification param
    double static_velocity_threshold_;
    int static_num_history_threshold_;

    // paramteter
    double gating_threshold_;

    ros::NodeHandle node_handle_;
    ros::Subscriber sub_detected_array_;
    ros::Publisher pub_object_array_;

    std_msgs::Header input_header_;

    double merge_distance_threshold_;
    const double CENTROID_DISTANCE = 0.05;

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
    void staticClassification();
    void makeOutput(const uav_msgs::DetectedObjectArray& input, const std::vector<bool>& matching_vec, 
                    uav_msgs::DetectedObjectArray& detected_objects_output);
    void removeUnnecessaryTarget();
    uav_msgs::DetectedObjectArray removeRedundantObjects(const uav_msgs::DetectedObjectArray& in_detected_objects, 
                                                         const std::vector<size_t> in_tracker_indices);


    // Helper
    bool isPointInPool(const std::vector<geometry_msgs::Point>& in_pool, 
                              const geometry_msgs::Point& in_point);
    bool arePointsEqual(const geometry_msgs::Point& in_point_a, const geometry_msgs::Point& in_point_b);
    bool arePointsClose(const geometry_msgs::Point& in_point_a, const geometry_msgs::Point& in_point_b, float in_radius);




public:
    KFTracker();
    virtual ~KFTracker();
    void run();


};


#endif