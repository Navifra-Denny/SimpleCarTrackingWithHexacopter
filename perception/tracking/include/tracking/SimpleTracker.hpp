#ifndef __SIMPLETRACKER_HPP__
#define __SIMPLETRACKER_HPP__

#include <iostream>
#include <vector>
#include "opencv2/video/tracking.hpp"

#include "tracking/TypeInfo.hpp"
#include "tracking/UtilityH.hpp"

class KFTrackV
{
private:
    cv::KalmanFilter m_filter;



};





class SimpleTracker
{
public:
    std::vector<DetectedObject> m_DetectedObjects;
    

    double m_dt;
    double m_MAX_ASSOCIATION_DISTANCE;
    double m_MAX_ASSOCIATION_SIZE_DIFF;
    double m_MAX_ASSOCIATION_ANGLE_DIFF;

    void DoOneStep(const std::vector<DetectedObject>& obj_list);

    SimpleTracker();
    virtual ~SimpleTracker();

private:
    long iTrackNumber;
    timespec m_TrackTimer;
    std::vector<KFTrackV> m_TrackSimply;


    void AssociateSimply();

};







#endif