#include "tracking2/KFTracker.hpp"

#define __APP_NAME__ "kfTracker"

KFTracker::KFTracker()
    : target_id_(0), 
      init_(false)
{
    GetParam();
}

KFTracker::~KFTracker()
{

}

void KFTracker::GetParam()
{
    node_handle_.getParam("kfTracker/gating_threshold", gating_threshold_);
    ROS_INFO("[%s] gating_threshold: %f", __APP_NAME__, gating_threshold_);

}

void KFTracker::run()
{
    sub_detected_array_ = node_handle_.subscribe("objects_in", 1, &KFTracker::callback, this);
}

void KFTracker::callback(const uav_msgs::DetectedObjectArray& input)
{
    input_header_ = input.header;
    // std::cout << input_header_ << std::endl;
    uav_msgs::DetectedObjectArray detected_objects_output;   
    tracker(input, detected_objects_output);


}

void KFTracker::tracker(const uav_msgs::DetectedObjectArray& input, 
                        uav_msgs::DetectedObjectArray& detected_objects_output)
{
    double timestamp = input.header.stamp.toSec();
    std::vector<bool> matching_vec(input.objects.size(), false);

    if(!init_)
    {
        initTracker(input, timestamp);
        return;
    }

    double dt = (timestamp - timestamp_);       // 대략 0.09 ~ 0.10 초씩 tracking update
    timestamp_ = timestamp;

    // Start KF process
    for (size_t i = 0; i < targets_.size(); i++)     // 초기 target_size = 3 (OS1_LiDAR bag file 기준)
    {
        targets_[i].is_stable_ = false;
        targets_[i].is_static_ = false;

        if (targets_[i].tracking_num_ == TrackingState::Die) continue;
  
        targets_[i].prediction(dt);

        std::vector<uav_msgs::DetectedObject> object_vec;
        bool success = dataAssociation(input, dt, matching_vec, object_vec, targets_[i]);
        if(!success) continue;

        targets_[i].update(object_vec);
    }
    // end KF process

    // making new kf target for no data association objects
    makeNewTargets(timestamp, input, matching_vec);

    // static dynamic classification
    

}


void KFTracker::initTracker(const uav_msgs::DetectedObjectArray& input, double timestamp)
{
    for (size_t i = 0; i < input.objects.size(); i++)
    {
        double px = input.objects[i].pose.position.x;
        double py = input.objects[i].pose.position.y;
        Eigen::VectorXd init_meas = Eigen::VectorXd(2);
        init_meas << px, py;

        // std::cout << px << ", " << py << std::endl;
        // std::cout << init_meas(0) << ", " << init_meas(1) << std::endl;

        KF kf;
        kf.initialize(init_meas, timestamp, target_id_);
        targets_.push_back(kf);
        target_id_++;
    }

    timestamp_ = timestamp;
    init_ = true;

}

bool KFTracker::dataAssociation(const uav_msgs::DetectedObjectArray& input, 
                        const double dt, std::vector<bool>& matching_vec, 
                        std::vector<uav_msgs::DetectedObject>& object_vec, 
                        KF& target)
{
    bool success = true;
    Eigen::VectorXd measure_pre;
    measure_pre = target.measure_pre_;
    
    bool is_second_init;
    if (target.tracking_num_ == TrackingState::Init) is_second_init = true;
    else is_second_init = false;

    measurementValidation(input, target, measure_pre, object_vec, matching_vec);    // target와 일정 distance 이내 object association

    // second detection for a target: update v
    if (is_second_init)
    {
        secondInit(target, object_vec, dt);
        success = false;
        return success;
    }

    updateTargetWithAssociatedObject(object_vec, target);

    if(target.tracking_num_ == TrackingState::Die) 
    {
        success = false;
        return success;
    }
    return success;
}

void KFTracker::measurementValidation(const uav_msgs::DetectedObjectArray& input, KF& target,
                                      const Eigen::VectorXd& measure_pre, 
                                      std::vector<uav_msgs::DetectedObject>& object_vec,
                                      std::vector<bool>& matching_vec)
{
    double d_x = 0, d_y = 0, d = 0;

    bool exists_closest_obj = false;
    double d_closest = std::numeric_limits<double>::max();
    double i_closest_obj = 0;

    std::cout << "measure_pre_ : " << measure_pre << std::endl;        

    for (size_t i = 0; i < input.objects.size(); i++)
    {
        double x = input.objects[i].pose.position.x;
        double y = input.objects[i].pose.position.y;

        Eigen::VectorXd meas = Eigen::VectorXd(2);
        meas << x, y;
        std::cout << "meas : " << meas << std::endl;        

        d_x = meas(0) - measure_pre(0);
        d_y = meas(1) - measure_pre(1);
        d = hypot(d_x, d_y);

        if (d < gating_threshold_)
        {
            if (d < d_closest)
            {
                d_closest = d;
                target.object_ = input.objects[i];
                i_closest_obj = i;
                exists_closest_obj = true;
            }
        }
    }

    if(exists_closest_obj)
    {
        matching_vec[i_closest_obj] = true;
        object_vec.push_back(target.object_);
    }
}

void KFTracker::secondInit(KF& target, const std::vector<uav_msgs::DetectedObject>& object_vec, double dt)
{
    if(object_vec.size() == 0)
    {
        target.tracking_num_ = TrackingState::Die;
        return;
    }

    std::cout << "target.init_meas_ : " << target.init_meas_ << std::endl;
    // record init measurement 
    double target_x = object_vec[0].pose.position.x;
    double target_y = object_vec[0].pose.position.y;
    double target_diff_x = target_x - target.init_meas_(0);
    double target_diff_y = target_y - target.init_meas_(1);
    double target_yaw = atan2(target_diff_y, target_diff_x);
    double dist = sqrt(target_diff_x * target_diff_x + target_diff_y * target_diff_y);
    double target_v = dist/dt;

    std::cout << "object_vec : " << target_x << ", " << target_y << std::endl;
    std::cout << "target_diff_x : " << target_diff_x << std::endl;    
    std::cout << "target_diff_y : " << target_diff_y << std::endl;    
    std::cout << "target_yaw : " << target_yaw << std::endl;
    std::cout << "target_v : " << target_v << std::endl;
    
    while (target_yaw > M_PI) target_yaw -= 2. * M_PI;
    while (target_yaw < -M_PI) target_yaw += 2. * M_PI;

    std::cout << "modified target_yaw : " << target_yaw << std::endl;

    target.tracking_num_++;
    std::cout << "tracking_num : " << target.tracking_num_ << std::endl;
    return;
}

void KFTracker::updateTargetWithAssociatedObject(const std::vector<uav_msgs::DetectedObject>& object_vec, KF& target)
{
    target.lifetime_++;
    if (!target.object_.label.empty() && target.object_.label != "unknown") target.label_ = target.object_.label;

    updateTrackingNum(object_vec, target);

    if(target.tracking_num_ == TrackingState::Stable || target.tracking_num_ == TrackingState::Occlusion) target.is_stable_ = true;

}

void KFTracker::updateTrackingNum(const std::vector<uav_msgs::DetectedObject>& object_vec, KF& target)
{
    
    std::cout << "변경 전 : " << target.tracking_num_ << std::endl;
    if(object_vec.size() > 0)
    {
        if (target.tracking_num_ < TrackingState::Stable) target.tracking_num_++;
        else if (target.tracking_num_ == TrackingState::Stable) target.tracking_num_ = TrackingState::Stable;
        else if (target.tracking_num_ > TrackingState::Stable && target.tracking_num_ < TrackingState::Lost) target.tracking_num_ = TrackingState::Stable;
        else if (target.tracking_num_ == TrackingState::Lost) target.tracking_num_ = TrackingState::Die;
    }
    else
    {
        if(target.tracking_num_ < TrackingState::Stable) target.tracking_num_ = TrackingState::Die;
        else if (target.tracking_num_ >= TrackingState::Stable && target.tracking_num_ < TrackingState::Lost) target.tracking_num_ ++;
        else if (target.tracking_num_ == TrackingState::Lost) target.tracking_num_ = TrackingState::Die;
    }

    std::cout << "변경 후 : " << target.tracking_num_ << std::endl;
}

void KFTracker::makeNewTargets(const double timestamp, const uav_msgs::DetectedObjectArray& input, 
                               const std::vector<bool>& matching_vec)
{
    for (size_t i = 0; i < input.objects.size(); i++)
    {
        if (matching_vec[i] == false)
        {
            double px = input.objects[i].pose.position.x;
            double py = input.objects[i].pose.position.y;
            Eigen::VectorXd init_meas = Eigen::VectorXd(2);
            init_meas << px, py;

            KF kf;
            kf.initialize(init_meas, timestamp, target_id_);
            kf.object_ = input.objects[i];
            targets_.push_back(kf);
            target_id_++;

            ROS_WARN_STREAM("응애");

        }
    }
}