#include <tuple>
#include <ros/ros.h>
#include "Utils/VisualSplinesService.hpp"
#include "bitbots_splines/LinearSpline.hpp"
#include "bitbots_splines/CubicSpline.hpp"
#include "bitbots_splines/SmoothSpline.hpp"

std::unique_ptr<bitbots_splines::LinearSpline> create_linear_spline(std::vector<double> times, std::vector<double> positions)
{
    if (times.size() != positions.size())
    {
        return NULL;
    }

    std::unique_ptr<bitbots_splines::LinearSpline> spline(new bitbots_splines::LinearSpline());

    for(auto i = 0; i < times.size(); ++i)
    {
        spline->addPoint(times[i], positions[i]);
    }

    return spline;
}

std::vector<std::unique_ptr<bitbots_splines::LinearSpline>> create_linear_splines(std::vector<double> times,
                                                                                    std::vector<double> points_x,
                                                                                    std::vector<double> points_y,
                                                                                    std::vector<double> points_z)
{
    std::vector<std::unique_ptr<bitbots_splines::LinearSpline>> ret;

    ret.push_back(create_linear_spline(times, points_x));
    ret.push_back(create_linear_spline(times, points_y));
    ret.push_back(create_linear_spline(times, points_z));

    return ret;
}

std::unique_ptr<bitbots_splines::CubicSpline> create_cubic_spline(std::vector<double> times, std::vector<double> positions, std::vector<double> velocities)
{
    if (times.size() != positions.size() || times.size() != velocities.size())
    {
        return NULL;
    }

    std::unique_ptr<bitbots_splines::CubicSpline> spline(new bitbots_splines::CubicSpline());

    for(auto i = 0; i < times.size(); ++i)
    {
        spline->addPoint(times[i], positions[i], velocities[i]);
    }

    return spline;
}

std::vector<std::unique_ptr<bitbots_splines::CubicSpline>> create_cubic_splines(std::vector<double> times,
                                                                                    std::vector<double> points_x,
                                                                                    std::vector<double> points_y,
                                                                                    std::vector<double> points_z,
                                                                                    std::vector<double> velocities_x,
                                                                                    std::vector<double> velocities_y,
                                                                                    std::vector<double> velocities_z)
{
    std::vector<std::unique_ptr<bitbots_splines::CubicSpline>> ret;

    ret.push_back(create_cubic_spline(times, points_x, velocities_x));
    ret.push_back(create_cubic_spline(times, points_y, velocities_y));
    ret.push_back(create_cubic_spline(times, points_z, velocities_z));

    return ret;
}

std::unique_ptr<bitbots_splines::SmoothSpline> create_smooth_spline(std::vector<double> times, std::vector<double> positions, std::vector<double> velocities, std::vector<double> accelerations)
{
    if (times.size() != positions.size() || times.size() != velocities.size())
    {
        return NULL;
    }

    std::unique_ptr<bitbots_splines::SmoothSpline> spline(new bitbots_splines::SmoothSpline());

    for(auto i = 0; i < times.size(); ++i)
    {
        spline->addPoint(times[i], positions[i], velocities[i], accelerations[i]);
    }

    return spline;
}

std::vector<std::unique_ptr<bitbots_splines::SmoothSpline>> create_smooth_splines(std::vector<double> times,
                                                                                    std::vector<double> points_x,
                                                                                    std::vector<double> points_y,
                                                                                    std::vector<double> points_z,
                                                                                    std::vector<double> velocities_x,
                                                                                    std::vector<double> velocities_y,
                                                                                    std::vector<double> velocities_z,
                                                                                    std::vector<double> accelerations_x,
                                                                                    std::vector<double> accelerations_y,
                                                                                    std::vector<double> accelerations_z)
{
    std::vector<std::unique_ptr<bitbots_splines::SmoothSpline>> ret;

    ret.push_back(create_smooth_spline(times, points_x, velocities_x, accelerations_x));
    ret.push_back(create_smooth_spline(times, points_y, velocities_y, accelerations_y));
    ret.push_back(create_smooth_spline(times, points_z, velocities_z, accelerations_z));

    return ret;
}

void visualize_spline_point(VisualSplinesService & vs_service,
                            std::string const str_namespace,
                            Color const color,
                            bitbots_splines::Spline * sp_x,
                            bitbots_splines::Spline * sp_y,
                            bitbots_splines::Spline * sp_z,
                            double const step)
{
     visualization_msgs::Marker vs_marker_points, vs_marker_lines;

    vs_service.set_marker(vs_marker_points, str_namespace, 0, visualization_msgs::Marker::POINTS);
    vs_service.set_marker_scale(vs_marker_points, 0.3, 0.3, 0.3);
    vs_service.set_marker_color(vs_marker_points, color);

    vs_service.set_marker(vs_marker_lines, str_namespace, 1, visualization_msgs::Marker::LINE_STRIP);
    vs_service.set_marker_scale(vs_marker_lines, 0.1, 0.1, 0.1);
    vs_service.set_marker_color(vs_marker_lines, color);

    vs_service.set_marker_position(vs_marker_points, vs_marker_lines, 100, sp_x, sp_y, sp_z, step);

    vs_service.publish_marker(vs_marker_points);
    vs_service.publish_marker(vs_marker_lines);
}

int main(int argc, char** argv)
{
    std::vector<double> times{0, 5, 10};
    std::vector<double> positions_x{0, 5, 10};
    std::vector<double> positions_y{10, 0, 10};
    std::vector<double> positions_z{0, 0, 0};
    std::vector<double> velocities_x{0, 0, 0};
    std::vector<double> velocities_y{-4, 0, 4};
    std::vector<double> velocities_z{0, 0, 0};
    std::vector<double> accelerations_x{0, 0, 0};
    std::vector<double> accelerations_y{0.8, 0.8, 0.8};
    std::vector<double> accelerations_z{0, 0, 0};

    auto vec_linear_splines = create_linear_splines(times,
                                                    positions_x,
                                                    positions_y,
                                                    positions_z);

    auto vec_cubic_splines = create_cubic_splines(times,
                                                    positions_x, positions_y, positions_z,
                                                    velocities_x,velocities_y, velocities_z);
                                                    
    auto vec_smooth_splines = create_smooth_splines(times,
                                                    positions_x, positions_y, positions_z,
                                                    velocities_x, velocities_y, velocities_z,
                                                    accelerations_x, accelerations_y, accelerations_z);

    VisualSplinesService vs_service(argc, argv, "spline_shapes", "visualization_marker");
    vs_service.set_marker_frame("/spline_visual_frame");
    ros::Rate rate(1);

    while(ros::ok())
    {
        visualize_spline_point(vs_service,
                                "linear_spline_shapes",
                                Color::red,
                                vec_linear_splines[0].get(),
                                vec_linear_splines[1].get(),
                                vec_linear_splines[2].get(),
                                0.1);
        
        visualize_spline_point(vs_service,
                                "cubic_spline_shapes",
                                Color::green,
                                vec_cubic_splines[0].get(),
                                vec_cubic_splines[1].get(),
                                vec_cubic_splines[2].get(),
                                0.1);

        visualize_spline_point(vs_service,
                                "smooth_spline_shapes",
                                Color::blue,
                                vec_smooth_splines[0].get(),
                                vec_smooth_splines[1].get(),
                                vec_smooth_splines[2].get(),
                                0.1);

        rate.sleep();
    }
}