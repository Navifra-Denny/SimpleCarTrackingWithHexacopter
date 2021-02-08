#include "tracking/UtilityH.hpp"

#include <iostream>
#include <sstream>
#include <string.h>

using namespace std;

UtilityH::UtilityH()
{
}

UtilityH::~UtilityH()
{
}

double UtilityH::FixNegativeAngle(const double& a)
{
    double angle = 0;
    if (a < -2.0 * M_PI || a >= 2.0 * M_PI) {
        angle = fmod(a, 2.0 * M_PI);
    }
    else angle = a;

    if(angle < 0){
        angle = 2.0 * M_PI + angle;
    }

    return angle;
}