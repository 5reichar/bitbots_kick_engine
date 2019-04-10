#include <tuple>
#include <ros/ros.h>
#include "Utils/VisualSplinesService.hpp"
#include "bitbots_splines/LinearSpline.hpp"
#include "bitbots_splines/CubicSpline.hpp"
#include "bitbots_splines/SmoothSpline.hpp"



template<typename Spline> std::unique_ptr<Spline> create_linear_spline(std::vector<double> times, std::vector<double> positions)
{
    if (times.size() != positions.size())
    {
        return NULL;
    }

    std::unique_ptr<Spline> spline(new Spline());

    for(auto i = 0; i < times.size(); ++i)
    {
        spline->addPoint(times[i], positions[i]);
    }

    return spline;
}

void visualize_spline_point(VisualSplinesService & vs_service,
                        visualization_msgs::Marker & vs_marker,
                        std::string const str_namespace,
                        bitbots_splines::Spline * sp_x,
                        bitbots_splines::Spline * sp_y,
                        bitbots_splines::Spline * sp_z)
{
    vs_service.set_marker(vs_marker, str_namespace, 0);
    vs_service.set_marker_properties(vs_marker, sp_x, sp_y, sp_z);
    vs_service.publish_marker(vs_marker);
}

int main(int argc, char** argv)
{
    std::vector<double> times{0, 5, 10, 15};
    std::vector<double> positions_x{0, 1, 1, 1};
    std::vector<double> positions_y{0, 0, 1, 1};
    std::vector<double> positions_z{0, 0, 0, 1};

    auto ap_linear_spline_x = create_linear_spline<bitbots_splines::LinearSpline>(times, positions_x);
    auto ap_linear_spline_y = create_linear_spline<bitbots_splines::LinearSpline>(times, positions_y);
    auto ap_linear_spline_z = create_linear_spline<bitbots_splines::LinearSpline>(times, positions_z);

    auto ap_cubic_spline_x = create_linear_spline<bitbots_splines::CubicSpline>(times, positions_x);
    auto ap_cubic_spline_y = create_linear_spline<bitbots_splines::CubicSpline>(times, positions_y);
    auto ap_cubic_spline_z = create_linear_spline<bitbots_splines::CubicSpline>(times, positions_z);

    auto ap_smooth_spline_x = create_linear_spline<bitbots_splines::SmoothSpline>(times, positions_x);
    auto ap_smooth_spline_y = create_linear_spline<bitbots_splines::SmoothSpline>(times, positions_y);
    auto ap_smooth_spline_z = create_linear_spline<bitbots_splines::SmoothSpline>(times, positions_z);

    VisualSplinesService vs_service(argc, argv, "spline_shapes", "visualization_marker");
    vs_service.set_marker_frame("/spline_visual_frame");
    ros::Rate rate(1);

    while(ros::ok())
    {
        visualization_msgs::Marker marker;

        //visualize_spline_point(vs_service, marker, "linear_spline_shapes", ap_linear_spline_x.get(), ap_linear_spline_y.get(), ap_linear_spline_z.get());
        visualize_spline_point(vs_service, marker, "cubic_spline_shapes", ap_cubic_spline_x.get(), ap_cubic_spline_y.get(), ap_cubic_spline_z.get());
        //visualize_spline_point(vs_service, marker, "smooth_spline_shapes", ap_smooth_spline_x.get(), ap_smooth_spline_y.get(), ap_smooth_spline_z.get());

        rate.sleep();
    }
}