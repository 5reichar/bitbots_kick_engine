#include <tuple>
#include <ros/ros.h>
#include "visualisation/visual_splines_service.h"
#include "spline/linear_spline.h"
#include "spline/cubic_spline.h"
#include "spline/smooth_spline.h"
#include "spline/beziercurve.h"

namespace bitbots_splines{
    template<class c>
    void testing(){
        std::shared_ptr<PoseHandle> pose = std::make_shared<PoseHandle>(std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>());
    }

    void add_points(VisualSplinesMaterial * vs_material){
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

    template<class c>
    VisualSplinesMaterial * visualize_curve(std::string s_namespace, Color color, double scale){
        std::shared_ptr<PoseHandle> pose = std::make_shared<PoseHandle>(std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>());

        auto * vsm_linear_spline = new VisualSplinesMaterial(pose);

        vsm_linear_spline->set_namspace(s_namespace);
        vsm_linear_spline->set_color(color);
        vsm_linear_spline->set_scale(scale);
        add_points(vsm_linear_spline);

        return vsm_linear_spline;
    }

    VisualSplinesMaterial * visualize_linear_spline(){
        return visualize_curve<LinearSpline>("linear_spline_shapes", Color::red, 0.1);
    }

    VisualSplinesMaterial * visualize_cubic_spline(){
        return visualize_curve<CubicSpline>("cubic_spline_shapes", Color::green, 0.1);
    }

    VisualSplinesMaterial * visualize_smooth_spline(){
        return visualize_curve<SmoothSpline>("smooth_spline_shapes", Color::blue, 0.1);
    }

    VisualSplinesMaterial * visualize_beziercurve(){
        return visualize_curve<Beziercurve>("bezier_curve_shapes", Color::yellow, 0.1);
    }
}

int main(int argc, char **argv)
{
    auto vsm_linear_spline = bitbots_splines::visualize_linear_spline();
    auto vsm_cubic_spline = bitbots_splines::visualize_cubic_spline();
    auto vsm_smooth_spline = bitbots_splines::visualize_smooth_spline();
    auto vsm_bezier_curve = bitbots_splines::visualize_beziercurve();

    bitbots_splines::VisualSplinesService vs_service(argc, argv, "spline_shapes", "visualization_marker");
    vs_service.set_marker_frame("/spline_visual_frame");
    ros::Rate rate(1);

    while (ros::ok())
    {
        vs_service.visualize_curve(vsm_linear_spline);
        vs_service.visualize_curve(vsm_cubic_spline);
        vs_service.visualize_curve(vsm_smooth_spline);
        vs_service.visualize_curve(vsm_bezier_curve);

        vs_service.visualize_points(vsm_linear_spline, "points", bitbots_splines::Color::white);

        rate.sleep();
    }

    delete vsm_linear_spline;
    delete vsm_cubic_spline;
    delete vsm_smooth_spline;
    delete vsm_bezier_curve;
}