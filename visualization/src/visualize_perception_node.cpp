#include "visualization/visualize_perception.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize_perception_node");
    VisualizeDetectedObjects app;

    ros::spin();

    return 0;
}