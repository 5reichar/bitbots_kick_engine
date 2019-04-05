#include <ros/ros.h>
#include "Utils/VisualSplinesService.hpp"

int main(int argc, char** argv)
{
    VisualSplinesService vs_service(argc, argv);
    ros::Rate rate(1);

    while(ros::ok())
    {
        auto marker = vs_service.get_marker("spline_shapes", 0);
        vs_service.set_marker_properties(marker);
        vs_service.publish_marker(marker);
        rate.sleep();
    }
}
