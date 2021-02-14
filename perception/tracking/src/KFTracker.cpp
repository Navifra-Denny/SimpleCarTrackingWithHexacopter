#include "tracking/KFTracker.hpp"

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
    node_handle_.getParam("kfTracker/life_time_threshold", life_time_threshold_);
    ROS_INFO("[%s] life_time_threshold: %d", __APP_NAME__, life_time_threshold_);
    node_handle_.getParam("kfTracker/static_num_history_threshold", static_num_history_threshold_);
    ROS_INFO("[%s] static_num_history_threshold: %d", __APP_NAME__, static_num_history_threshold_);
    node_handle_.getParam("kfTracker/static_velocity_threshold", static_velocity_threshold_);
    ROS_INFO("[%s] static_velocity_threshold: %f", __APP_NAME__, static_velocity_threshold_);
    node_handle_.getParam("kfTracker/merge_distance_threshold", merge_distance_threshold_);
    ROS_INFO("[%s] merge_distance_threshold: %f", __APP_NAME__, merge_distance_threshold_);



}

void KFTracker::run()
{
    pub_object_array_ = node_handle_.advertise<uav_msgs::DetectedObjectArray>("objects_out", 1);
    sub_detected_array_ = node_handle_.subscribe("objects_in", 1, &KFTracker::callback, this);
}

void KFTracker::callback(const uav_msgs::DetectedObjectArray& input)
{
    input_header_ = input.header;
    // std::cout << input_header_ << std::endl;
    uav_msgs::DetectedObjectArray detected_objects_output;   
    tracker(input, detected_objects_output);

    pub_object_array_.publish(detected_objects_output);

}

void KFTracker::tracker(const uav_msgs::DetectedObjectArray& input, 
                        uav_msgs::DetectedObjectArray& detected_objects_output)
{
    double timestamp = input.header.stamp.toSec();
    std::vector<bool> matching_vec(input.objects.size(), false);

    if(!init_)
    {
        initTracker(input, timestamp);
        makeOutput(input, matching_vec, detected_objects_output);
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
    staticClassification();

    // making output for visualization
    makeOutput(input, matching_vec, detected_objects_output);

    // remove unnecessary kf object
    removeUnnecessaryTarget();

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

    target.vel_ = target_v;
    target.yaw_ = target_yaw;

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

            ROS_WARN("New track id: %d", target_id_);
        }
    }
}

void KFTracker::staticClassification()
{
    for (size_t i = 0; i < targets_.size(); i++)
    {
        double v = hypot(targets_[i].state_post_(2), targets_[i].state_post_(3));
        double current_velocity = std::abs(v);
        
        // ROS_ERROR_STREAM(current_velocity);
        
        targets_[i].vel_history_.push_back(current_velocity);
        if(targets_[i].tracking_num_ == TrackingState::Stable && targets_[i].lifetime_ > life_time_threshold_)
        {
            int index = 0;
            double sum_vel = 0;
            double avg_vel = 0;

            for (auto rit = targets_[i].vel_history_.rbegin(); index < static_num_history_threshold_; ++rit)
            {
                index++;
                sum_vel += *rit;
            }

            avg_vel = double(sum_vel / static_num_history_threshold_);

            if(avg_vel < static_velocity_threshold_ && current_velocity < static_velocity_threshold_)
            {
                targets_[i].is_static_ = true;
            }

        }

    }
}

void KFTracker::removeUnnecessaryTarget()
{
    std::vector<KF> temp_targets;
    for (size_t i = 0; i < targets_.size(); i++)
    {
        if (targets_[i].tracking_num_ != TrackingState::Die)
        {
            temp_targets.push_back(targets_[i]);
        }
    }

    std::vector<KF>().swap(targets_);
    targets_ = temp_targets;

}

void KFTracker::makeOutput(const uav_msgs::DetectedObjectArray& input, 
                           const std::vector<bool>& matching_vec, 
                           uav_msgs::DetectedObjectArray& detected_objects_output)
{
    uav_msgs::DetectedObjectArray tmp_objects;
    tmp_objects.header = input.header;
    std::vector<size_t> used_targets_indices;
    for (size_t i = 0; i < targets_.size(); i++)
    {
        double tx = targets_[i].state_post_(0);
        double ty = targets_[i].state_post_(1);
        double vx = targets_[i].state_post_(2);
        double vy = targets_[i].state_post_(3);

        double tv = targets_[i].vel_;
        double tyaw = targets_[i].yaw_;

        while (tyaw > M_PI) tyaw -= 2. * M_PI;
        while (tyaw < -M_PI) tyaw += 2. * M_PI;

        tf::Quaternion q = tf::createQuaternionFromYaw(tyaw);

        uav_msgs::DetectedObject dd;
        dd = targets_[i].object_;
        dd.id = targets_[i].kf_id_;
        dd.velocity.linear.x = vx;
        dd.velocity.linear.y = vy;
        dd.velocity_reliable = targets_[i].is_stable_;
        dd.pose_reliable = targets_[i].is_stable_;

        if (!targets_[i].is_static_ && targets_[i].is_stable_)
        {
            if(targets_[i].object_.dimensions.x < targets_[i].object_.dimensions.y)
            {
                dd.dimensions.x = targets_[i].object_.dimensions.y;
                dd.dimensions.y = targets_[i].object_.dimensions.x;
            }

            // posteriori state 
            dd.pose.position.x = tx;
            dd.pose.position.y = ty;

            if (!std::isnan(q[0])) dd.pose.orientation.x = q[0];
            if (!std::isnan(q[1])) dd.pose.orientation.y = q[1];
            if (!std::isnan(q[2])) dd.pose.orientation.z = q[2];
            if (!std::isnan(q[3])) dd.pose.orientation.w = q[3];

        }
        
        if (targets_[i].is_stable_ || (targets_[i].tracking_num_ >= TrackingState::Init && targets_[i].tracking_num_ < TrackingState::Stable))
        {
            tmp_objects.objects.push_back(dd);
            used_targets_indices.push_back(i);
        }

        detected_objects_output =removeRedundantObjects(tmp_objects, used_targets_indices);

    }
}

bool KFTracker::arePointsClose(const geometry_msgs::Point& in_point_a, const geometry_msgs::Point& in_point_b, float in_radius)
{
    return (fabs(in_point_a.x - in_point_b.x) <= in_radius) && (fabs(in_point_a.y - in_point_b.y) <= in_radius);
}

bool KFTracker::arePointsEqual(const geometry_msgs::Point& in_point_a, const geometry_msgs::Point& in_point_b)
{
    return arePointsClose(in_point_a, in_point_b, CENTROID_DISTANCE);
}

bool KFTracker::isPointInPool(const std::vector<geometry_msgs::Point>& in_pool, 
                              const geometry_msgs::Point& in_point)
{
    for(size_t j = 0; j < in_pool.size(); j++)
    {
        if(arePointsEqual(in_pool[j], in_point)){
            return true;
        }
    }
    return false;
}

uav_msgs::DetectedObjectArray KFTracker::removeRedundantObjects(const uav_msgs::DetectedObjectArray& in_detected_objects, 
                                                                const std::vector<size_t> in_tracker_indices)
{
    if (in_detected_objects.objects.size() != in_tracker_indices.size()) 
        return in_detected_objects;

    uav_msgs::DetectedObjectArray resulting_objects;
    resulting_objects.header = in_detected_objects.header;

    std::vector<geometry_msgs::Point> centroids;

    for (size_t i = 0; i < in_detected_objects.objects.size(); i++)
    {
        if(!isPointInPool(centroids, in_detected_objects.objects[i].pose.position))
        {
            centroids.push_back(in_detected_objects.objects[i].pose.position);
        }
    }

    ROS_WARN("detected_objects_size : %ld", in_detected_objects.objects.size());
    ROS_WARN("centroids_size : %ld", centroids.size());

    std::vector<std::vector<size_t>> matching_objects(centroids.size());
    for(size_t k = 0; k < in_detected_objects.objects.size(); k++)
    {
        const auto& object = in_detected_objects.objects[k];
        for(size_t i = 0; i < centroids.size(); i++)
        {
            if(arePointsClose(object.pose.position, centroids[i], merge_distance_threshold_))
            {
                matching_objects[i].push_back(k);
            }
        }
    }

    // get oldest object on each point
    for(size_t i = 0; i < matching_objects.size(); i++)
    {
        size_t oldest_object_index = 0;
        int oldest_lifespan = -1;
        std::string best_label;

        for(size_t j = 0; j < matching_objects[i].size(); j++)
        {
            size_t current_index = matching_objects[i][j];
            int current_lifespan = targets_[in_tracker_indices[current_index]].lifetime_;
            if (current_lifespan > oldest_lifespan)
            {
                oldest_lifespan = current_lifespan;
                oldest_object_index = current_index;
            }
            if (!targets_[in_tracker_indices[current_index]].label_.empty() &&
                targets_[in_tracker_indices[current_index]].label_ != "unknown")
            {
                best_label = targets_[in_tracker_indices[current_index]].label_;
            }
        }

        for (size_t j = 0; j < matching_objects[i].size(); j++)
        {
            size_t current_index = matching_objects[i][j];
            if (current_index != oldest_object_index)
            {
                targets_[in_tracker_indices[current_index]].tracking_num_ = TrackingState::Die;
            }
        }

        uav_msgs::DetectedObject best_object;
        best_object = in_detected_objects.objects[oldest_object_index];
        if(best_label != "unknown" && !best_label.empty())
        {
            best_object.label = best_label;
        }

        resulting_objects.objects.push_back(best_object);
    }

    ROS_WARN("centroids_size : %ld", resulting_objects.objects.size());
    
    return resulting_objects;
}