#include "visualize_detected_objects.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization");
    VisualizeDetectedObjects app;

    ros::spin();

    return 0;
}