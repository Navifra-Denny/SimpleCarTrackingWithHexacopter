#include "tracking/UtilityH.hpp"

#include <iostream>
#include <sstream>
#include <string.h>

using namespace std;

namespace UtilityHNS
{

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

void UtilityH::GetTickCount(struct timespec& t)
{
  while(clock_gettime(0, & t) == -1);
}

double UtilityH::GetTimeDiffNow(const struct timespec& old_t)
{
  struct timespec curr_t;
  GetTickCount(curr_t);
  return (curr_t.tv_sec - old_t.tv_sec) + ((double)(curr_t.tv_nsec - old_t.tv_nsec)/ 1000000000.0);
}


}