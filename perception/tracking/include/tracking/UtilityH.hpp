#ifndef __UTILITYH_HPP__
#define __UTILITYH_HPP__

#include <assert.h>
#include <string>
#include <math.h>

namespace UtilityHNS
{

class UtilityH
{
public:
    UtilityH();
    virtual ~UtilityH();

    static double FixNegativeAngle(const double& a);
    static void GetTickCount(struct timespec& t);
    static double GetTimeDiffNow(const struct timespec& old_t);
};
}
#endif