#ifndef __SIMPLETRACKER_HPP__
#define __SIMPLETRACKER_HPP__

#include <iostream>
#include <vector>

const int SIMPLE_TRACKER = 1;


class SimpleTracker
{
public:
    double m_dt;
    double m_MAX_ASSOCIATION_DISTANCE;
    double m_MAX_ASSOCIATION_SIZE_DIFF;
    double m_MAX_ASSOCIATION_ANGLE_DIFF;
    bool m_bFirstCall;
    int m_nMinTrustAppearances;
    double m_Horizon;
    double m_CirclesResolution;
};







#endif