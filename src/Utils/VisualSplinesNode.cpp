#include <tuple>
#include <ros/ros.h>
#include "Utils/VisualSplinesService.hpp"
#include "bitbots_splines/LinearSpline.hpp"
#include "bitbots_splines/CubicSpline.hpp"
#include "bitbots_splines/SmoothSpline.hpp"
#include "bitbots_splines/Beziercurve.hpp"

void add_points(VisualSplinesMaterial *vs_material)
{
    vs_material->add_point_to_x(0.0, 0.0, 0.0, 0.0);
    vs_material->add_point_to_x(5.0, 5.0, 0.0, 0.0);
    vs_material->add_point_to_x(10.0, 10.0, 0.0, 0.0);

    vs_material->add_point_to_y(0.0, 10.0, -4.0, 0.8);
    vs_material->add_point_to_y(5.0, 0.0, 0.0, 0.8);
    vs_material->add_point_to_y(10.0, 10.0, 4.0, 0.8);

    vs_material->add_point_to_z(0.0, 0.0, 0.0, 0.0);
    vs_material->add_point_to_z(5.0, 0.0, 0.0, 0.0);
    vs_material->add_point_to_z(10.0, 0.0, 0.0, 0.0);
}

int main(int argc, char **argv)
{
    VisualSplinesMaterial *vsm_linear_spline;
    vsm_linear_spline = new VisualSplinesMaterial(new bitbots_splines::LinearSpline(),
                                                  new bitbots_splines::LinearSpline(),
                                                  new bitbots_splines::LinearSpline());
    vsm_linear_spline->set_namspace("linear_spline_shapes");
    vsm_linear_spline->set_color(Color::red);
    vsm_linear_spline->set_scale(0.1);
    add_points(vsm_linear_spline);

    VisualSplinesMaterial *vsm_cubic_spline;
    vsm_cubic_spline = new VisualSplinesMaterial(new bitbots_splines::CubicSpline(),
                                                 new bitbots_splines::CubicSpline(),
                                                 new bitbots_splines::CubicSpline());
    vsm_cubic_spline->set_namspace("cubic_spline_shapes");
    vsm_cubic_spline->set_color(Color::green);
    vsm_cubic_spline->set_scale(0.1);
    add_points(vsm_cubic_spline);

    VisualSplinesMaterial *vsm_smooth_spline;
    vsm_smooth_spline = new VisualSplinesMaterial(new bitbots_splines::SmoothSpline(),
                                                  new bitbots_splines::SmoothSpline(),
                                                  new bitbots_splines::SmoothSpline());
    vsm_smooth_spline->set_namspace("smooth_spline_shapes");
    vsm_smooth_spline->set_color(Color::blue);
    vsm_smooth_spline->set_scale(0.1);
    add_points(vsm_smooth_spline);

    VisualSplinesMaterial *vsm_bezier_curve;
    vsm_bezier_curve = new VisualSplinesMaterial(new bitbots_splines::Beziercurve(),
                                                 new bitbots_splines::Beziercurve(),
                                                 new bitbots_splines::Beziercurve());
    vsm_bezier_curve->set_namspace("bezier_curve_shapes");
    vsm_bezier_curve->set_color(Color::yellow);
    vsm_bezier_curve->set_scale(0.1);
    add_points(vsm_bezier_curve);

    VisualSplinesService vs_service(argc, argv, "spline_shapes", "visualization_marker");
    vs_service.set_marker_frame("/spline_visual_frame");
    ros::Rate rate(1);

    while (ros::ok())
    {
        vs_service.visualize_curve(vsm_linear_spline);
        vs_service.visualize_curve(vsm_cubic_spline);
        vs_service.visualize_curve(vsm_smooth_spline);
        vs_service.visualize_curve(vsm_bezier_curve);

        vs_service.visualize_points(vsm_linear_spline, "points", Color::white);

        rate.sleep();
    }

    delete vsm_linear_spline;
    delete vsm_cubic_spline;
    delete vsm_smooth_spline;
    delete vsm_bezier_curve;
}