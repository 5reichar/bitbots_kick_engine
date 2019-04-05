#include <ros/ros.h>
#include "Utils/VisualSplinesService.hpp"
#include "bitbots_splines/LinearSpline.hpp"
#include "bitbots_splines/CubicSpline.hpp"
#include "bitbots_splines/SmoothSpline.hpp"

template<typename Spline> Spline * create_linear_spline(std::vector<double> times, std::vector<double> positions)
{
    if (times.size() != positions.size())
    {
        return NULL;
    }

    auto spline = new Spline();

    for(auto i = 0; i < times.size(); ++i)
    {
        spline->addPoint(times[i], positions[i]);
    }

    return spline;
}

int main(int argc, char** argv)
{
    std::vector<double> times{0, 5, 10, 15};
    std::vector<double> positions_x{0, 1, 1, 1};
    std::vector<double> positions_y{0, 0, 1, 1};
    std::vector<double> positions_z{0, 0, 0, 1};

    std::unique_ptr<bitbots_splines::LinearSpline> ap_linear_spline_x(create_linear_spline<bitbots_splines::LinearSpline>(times, positions_x));
    std::unique_ptr<bitbots_splines::LinearSpline> ap_linear_spline_y(create_linear_spline<bitbots_splines::LinearSpline>(times, positions_y));
    std::unique_ptr<bitbots_splines::LinearSpline> ap_linear_spline_z(create_linear_spline<bitbots_splines::LinearSpline>(times, positions_z));

    std::unique_ptr<bitbots_splines::CubicSpline> ap_cubic_spline_x(create_linear_spline<bitbots_splines::CubicSpline>(times, positions_x));
    std::unique_ptr<bitbots_splines::CubicSpline> ap_cubic_spline_y(create_linear_spline<bitbots_splines::CubicSpline>(times, positions_y));
    std::unique_ptr<bitbots_splines::CubicSpline> ap_cubic_spline_z(create_linear_spline<bitbots_splines::CubicSpline>(times, positions_z));

    std::unique_ptr<bitbots_splines::SmoothSpline> ap_smooth_spline_x(create_linear_spline<bitbots_splines::SmoothSpline>(times, positions_x));
    std::unique_ptr<bitbots_splines::SmoothSpline> ap_smooth_spline_y(create_linear_spline<bitbots_splines::SmoothSpline>(times, positions_y));
    std::unique_ptr<bitbots_splines::SmoothSpline> ap_smooth_spline_z(create_linear_spline<bitbots_splines::SmoothSpline>(times, positions_z));

    VisualSplinesService vs_service(argc, argv, "spline_shapes", "visualization_marker");
    ros::Rate rate(1);

    while(ros::ok())
    {
        visualization_msgs::Marker marker;

        vs_service.set_marker(marker, "/spline_visual_frame", "linear_spline_shapes", 0);
        vs_service.set_marker_properties(marker, ap_linear_spline_x.get(), ap_linear_spline_y.get(), ap_linear_spline_z.get());
        vs_service.publish_marker(marker);

        vs_service.set_marker(marker, "/spline_visual_frame", "cubic_spline_shapes", 0);
        vs_service.set_marker_properties(marker, ap_cubic_spline_x.get(), ap_cubic_spline_y.get(), ap_cubic_spline_z.get());
        vs_service.publish_marker(marker);

        vs_service.set_marker(marker, "/spline_visual_frame", "smooth_spline_shapes", 0);
        vs_service.set_marker_properties(marker, ap_smooth_spline_x.get(), ap_smooth_spline_y.get(), ap_smooth_spline_z.get());
        vs_service.publish_marker(marker);
        rate.sleep();
    }
}