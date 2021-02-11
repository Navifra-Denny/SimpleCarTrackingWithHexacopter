#include "tracking/SimpleTracker.hpp"

#include <iostream>
#include <vector>
#include <cstdio>
#include <float.h>

SimpleTracker::SimpleTracker()
{
    iTrackNumber = 1;
    m_dt = 0.1;
    m_MAX_ASSOCIATION_DISTANCE = 2.0;
    m_MAX_ASSOCIATION_SIZE_DIFF = 5.0;
    m_MAX_ASSOCIATION_ANGLE_DIFF = 0.6;
}

SimpleTracker::~SimpleTracker()
{

}

void SimpleTracker::DoOneStep(const std::vector<DetectedObject>& obj_list)
{
    UtilityHNS::UtilityH::GetTickCount(m_TrackTimer);
    m_DetectedObjects = obj_list;

    AssociateSimply();





}

void SimpleTracker::AssociateSimply()
{
    for(unsigned int i = 0; i < m_TrackSimply.size(); i++)
        

}

