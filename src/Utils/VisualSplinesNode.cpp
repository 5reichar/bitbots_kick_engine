#include <ros/ros.h>
#include "Utils/VisualSplinesService.hpp"

int main(int argc, char** argv)
{
    VisualSplinesService vs_service(argc, argv, "spline_shapes", "visualization_marker");
    ros::Rate rate(1);

    while(ros::ok())
    {
        visualization_msgs::Marker marker;

        vs_service.set_marker(marker, "/spline_visual_frame", "spline_shapes", 0);
        vs_service.set_marker_properties(marker);
        vs_service.publish_marker(marker);
        rate.sleep();
    }
}