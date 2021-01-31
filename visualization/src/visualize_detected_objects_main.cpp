#include "visualization/visualize_detected_objects.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_detected_objects");
    VisualizeDetectedObjects app;

    ros::spin();

    return 0;
}