#include <euclideanClustering.hpp>
#include <rayGroundFilter.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "euclideanClustering");

    EuclideanClustering euclideanClustering;

    ros::spin();
}