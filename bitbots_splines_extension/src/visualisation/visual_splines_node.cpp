#include <tuple>
#include <ros/ros.h>
#include "visualisation/visual_splines_service.h"
#include "spline/linear_spline.h"
#include "spline/cubic_spline.h"
#include "spline/quintic_spline.h"
#include "spline/beziercurve.h"

namespace bitbots_splines{
    template<class c>
    void testing(){
        ROS_INFO_STREAM("Start Testing");
        std::shared_ptr<PoseHandle> pose = std::make_shared<PoseHandle>(std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>()
                                                                        , std::make_shared<c>());

        std::vector<std::pair<double, double>> test_values;
        test_values.emplace_back(std::make_pair<double, double>(0, 0));
        test_values.emplace_back(std::make_pair<double, double>(0.25, 0));
        test_values.emplace_back(std::make_pair<double, double>(0.5, 0));
        test_values.emplace_back(std::make_pair<double, double>(0.75, 5.49779));
        test_values.emplace_back(std::make_pair<double, double>(1, 5.49779));
        test_values.emplace_back(std::make_pair<double, double>(1.33333, 1.5708));
        test_values.emplace_back(std::make_pair<double, double>(1.66667, 1.5708));
        test_values.emplace_back(std::make_pair<double, double>(2, 2.35619));
        test_values.emplace_back(std::make_pair<double, double>(2.5, 1.5708));
        test_values.emplace_back(std::make_pair<double, double>(3, 5.75959));
        test_values.emplace_back(std::make_pair<double, double>(4, 0));

        for(auto it : test_values){
            pose->pitch()->add_point(Curve::Point{it.first, it.second});
        }

        int point = 0;
        int successful = 0;
        for(auto it : test_values){
            ++point;
            double t = pose->pitch()->position(it.first);
            if(std::abs(it.second - t) > 0.00001){
                ROS_INFO_STREAM("Check Failed (Point: " + std::to_string(point) + ", time: " + std::to_string(it.first) + ", position: " + std::to_string(it.second) + ", position-from-curve: " + std::to_string(t) + ")");
            }else{
                ++successful;
            }
        }

        ROS_INFO_STREAM("Finished Testing, [" + std::to_string(successful) + " out of " + std::to_string(point) + " successful]");
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

    void visualize(int argc, char **argv){
        auto vsm_linear_spline = visualize_curve<LinearSpline>("linear_spline_shapes", Color::red, 0.1);
        auto vsm_cubic_spline = visualize_curve<CubicSpline>("cubic_spline_shapes", Color::green, 0.1);
        auto vsm_smooth_spline = visualize_curve<QuinticSpline>("smooth_spline_shapes", Color::blue, 0.1);
        auto vsm_bezier_curve = visualize_curve<Beziercurve>("bezier_curve_shapes", Color::yellow, 0.1);

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
}

int main(int argc, char **argv)
{
    //bitbots_splines::visualize(argc, argv);

    bitbots_splines::testing<bitbots_splines::CubicSpline>();
}