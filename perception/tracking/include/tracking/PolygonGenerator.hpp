#ifndef __POLYGONGENERATOR_HPP__
#define __POLYGONGENERATOR_HPP__

#include "tracking/kfTrackerCore.hpp"


class QuarterView
{
public:
    int id;
    int min_ang;
    int max_ang;
    Point max_from_center;
    bool bFirst;

    QuarterView(const int& min_a, const int& max_a, const int& index){
        min_ang = min_a;
        max_ang = max_a;
        id = index;
        bFirst = true;
    }

    void InitQuarterView(const int& min_a, const int& max_a, const int& index){
        min_ang = min_a;
        max_ang = max_a;
        id = index;
        bFirst = true;
    }
    
    void ResetQuarterView(){
      bFirst = true;
    }
  
    bool UpdateQuarterView(const Point& v)
    {
        if(v.a <= min_ang || v.a > max_ang)
          return false;   

        if(bFirst)
        {
          max_from_center = v;
          bFirst = false;
        }
        else if(v.cost > max_from_center.cost)
          max_from_center = v;    

        return true;
    }

    bool GetMaxPoint(Point& maxPoint)
    {
        if(bFirst)
            return false;
        
        else
            maxPoint = max_from_center;    
       
        return true;
    }
};

class PolygonGenerator
{
public:
    Point m_centroid;
    std::vector<QuarterView> m_Quarters;
    std::vector<Point> m_Polygon;

    PolygonGenerator(int nQuarters);
    virtual ~PolygonGenerator();
    std::vector<QuarterView> CreateQuarterViews(const int& nResolution);
    std::vector<Point> EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, 
                                                        const Point& original_centroid, 
                                                        Point& new_centroid, 
                                                        const double& polygon_resolution = 1.0);


};


#endif