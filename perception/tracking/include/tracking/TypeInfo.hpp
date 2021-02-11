#ifndef __TYPEINFO_HPP__
#define __TYPEINFO_HPP__

#include <iostream>
#include <vector>

class Point
{
public:
    double x;
    double y;
    double z;
    double a;
    double cost;

    Point()
    {
        x = 0;
        y = 0;
        z = 0;
        a = 0;
        cost = 0;
    }

    Point(const double& x, const double& y, const double& z, const double& a)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->a = a;
        this->cost = 0;
    }
   
};

class DetectedObject
{
public:
    int id;
    std::string label;
    Point center;
    Point predicted_center;
    std::vector<Point> centers_list;
    std::vector<Point> contour;
    double w;
    double l;
    double h;
    double actual_yaw;

    DetectedObject()
    {
        id = 0;
        w = 0;
        l = 0;
        h = 0;
        actual_yaw = 0;
    }
};


#endif