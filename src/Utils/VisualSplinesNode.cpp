#include <tuple>
#include <ros/ros.h>
#include "Utils/VisualSplinesService.hpp"
#include "bitbots_splines/LinearSpline.hpp"
#include "bitbots_splines/CubicSpline.hpp"
#include "bitbots_splines/SmoothSpline.hpp"
#include "bitbots_splines/Beziercurve.hpp"

void visualize_spline_point(VisualSplinesService & vs_service,
                            std::string const str_namespace,
                            Color const color,
                            VisualSplinesMaterial * vs_material,
                            double const step,
                            bool const debug = false)
{
    visualization_msgs::Marker vs_marker_points, vs_marker_lines;

    vs_service.set_marker(vs_marker_points, str_namespace, 0, visualization_msgs::Marker::POINTS);
    vs_service.set_marker_scale(vs_marker_points, 0.3, 0.3, 0.3);
    vs_service.set_marker_color(vs_marker_points, color);

    vs_service.set_marker(vs_marker_lines, str_namespace, 1, visualization_msgs::Marker::LINE_STRIP);
    vs_service.set_marker_scale(vs_marker_lines, 0.1, 0.1, 0.1);
    vs_service.set_marker_color(vs_marker_lines, color);

    vs_service.set_marker_position(vs_marker_points, vs_marker_lines, 100, vs_material, step, debug);

    vs_service.publish_marker(vs_marker_points);
    vs_service.publish_marker(vs_marker_lines);
}

void add_points(VisualSplinesMaterial * vs_material)
{
    vs_material->add_point_to_x( 0.0,  0.0,  0.0,  0.0);
    vs_material->add_point_to_x( 5.0,  5.0,  0.0,  0.0);
    vs_material->add_point_to_x(10.0, 10.0,  0.0,  0.0);

    vs_material->add_point_to_y( 0.0, 10.0, -4.0,  0.8);
    vs_material->add_point_to_y( 5.0,  0.0,  0.0,  0.8);
    vs_material->add_point_to_y(10.0, 10.0,  4.0,  0.8);

    vs_material->add_point_to_z( 0.0,  0.0,  0.0,  0.0);
    vs_material->add_point_to_z( 5.0,  0.0,  0.0,  0.0);
    vs_material->add_point_to_z(10.0,  0.0,  0.0,  0.0);
}

int main(int argc, char** argv)
{
    VisualSplinesMaterial * vsm_linear_spline;
    vsm_linear_spline = new VisualSplinesMaterial(new bitbots_splines::LinearSpline(),
                                                    new bitbots_splines::LinearSpline(),
                                                    new bitbots_splines::LinearSpline());
    add_points(vsm_linear_spline);

    VisualSplinesMaterial * vsm_cubic_spline;
    vsm_cubic_spline = new VisualSplinesMaterial(new bitbots_splines::CubicSpline(),
                                                    new bitbots_splines::CubicSpline(),
                                                    new bitbots_splines::CubicSpline());
    add_points(vsm_cubic_spline);
    
    VisualSplinesMaterial * vsm_smooth_spline;
    vsm_smooth_spline = new VisualSplinesMaterial(new bitbots_splines::SmoothSpline(),
                                                    new bitbots_splines::SmoothSpline(),
                                                    new bitbots_splines::SmoothSpline());
    add_points(vsm_smooth_spline);

    VisualSplinesMaterial * vsm_bezier_curve;
    vsm_bezier_curve = new VisualSplinesMaterial(new Beziercurve(),
                                                    new Beziercurve(),
                                                    new Beziercurve());
    add_points(vsm_bezier_curve);

    VisualSplinesService vs_service(argc, argv, "spline_shapes", "visualization_marker");
    vs_service.set_marker_frame("/spline_visual_frame");
    ros::Rate rate(1);

    while(ros::ok())
    {
        visualize_spline_point(vs_service,
                                "linear_spline_shapes",
                                Color::red,
                                vsm_linear_spline,
                                0.1);
        
        visualize_spline_point(vs_service,
                                "cubic_spline_shapes",
                                Color::green,
                                vsm_cubic_spline,
                                0.1);

        visualize_spline_point(vs_service,
                                "smooth_spline_shapes",
                                Color::blue,
                                vsm_smooth_spline,
                                0.1);

        visualize_spline_point(vs_service,
                                "bezier_curve_shapes",
                                Color::blue,
                                vsm_bezier_curve,
                                0.1,
                                false);

        rate.sleep();
    }

    delete vsm_linear_spline;
    delete vsm_cubic_spline;
    delete vsm_smooth_spline;
}