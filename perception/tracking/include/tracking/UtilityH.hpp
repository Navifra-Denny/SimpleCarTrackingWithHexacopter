#ifndef __UTILITYH_HPP__
#define __UTILITYH_HPP__

#include <assert.h>
#include <string>
#include <math.h>

class UtilityH
{
public:
    UtilityH();
    virtual ~UtilityH();

    static double FixNegativeAngle(const double& a);
};

#endif