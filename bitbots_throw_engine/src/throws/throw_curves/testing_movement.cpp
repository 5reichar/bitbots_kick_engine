#include "throws/throw_curves/testing_movement.h"
#include "../../bitbots_splines_extension/include/spline/linear_spline.h"
#include "../../bitbots_splines_extension/include/spline/cubic_spline.h"
#include "../../bitbots_splines_extension/include/spline/smooth_spline.h"
#include "../../bitbots_splines_extension/include/spline/beziercurve.h"
#include "ros_interface/publisher/system_publisher.h"
#include <sstream>

namespace bitbots_throw{
    TestingMovement::TestingMovement(std::shared_ptr<ThrowMaterial> material)
        :ThrowMovement(material){
        max_prepare_rounds_ = 20;
    }

    double TestingMovement::init_material(){
        trajectory_time_ = 0.0;

        test_curves();
        move_arms_and_feet();
        squat();
        rotate_right_hand();
        curve_with_radian();

        return trajectory_time_;
    }

    void TestingMovement::test_curves(){
        test_curve_type<bitbots_splines::Beziercurve>("Beziercurve");
        test_curve_type<bitbots_splines::LinearSpline>("LinearSpline");
        test_curve_type<bitbots_splines::CubicSpline>("CubicSpline");

        // TODO: Find out why point are not getting added to Smooth Spline
        // test_curve_type<bitbots_splines::SmoothSpline>("SmoothSpline");

        SystemPublisher::publish_info("Finished Testing", "TestingMovement");
    }

    void TestingMovement::move_arms_and_feet(){
        SystemPublisher::publish_info("Start Testing Movement III", "TestingMovement");
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 3.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 3.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 3.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 3.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        SystemPublisher::publish_info("Finished Testing Movement III", "TestingMovement");
    }

    void TestingMovement::squat(){
        SystemPublisher::publish_info("Start Testing Movement II", "TestingMovement");
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        SystemPublisher::publish_info("Finished Testing Movement II", "TestingMovement");
    }

    void TestingMovement::rotate_right_hand(){
        SystemPublisher::publish_info("Start Testing Movement I", "TestingMovement");
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        SystemPublisher::publish_info("Finished Testing Movement I", "TestingMovement");
    }

    template<typename curve_type>
    void TestingMovement::test_curve_type(std::string const & type_name){
        std::stringstream log;
        bitbots_splines::Curve * curve;
        SystemPublisher system_publisher;
        std::vector<std::pair<double, double>> values;
        SystemPublisher::publish_info("Testing " + type_name, "TestingMovement");


        log << "asc. points, -3.0, -1.0, 0.0, 1.0, 3.0" << std::endl;
        values.emplace_back(std::make_pair<double, double>(0.0, -3.0));
        values.emplace_back(std::make_pair<double, double>(0.5, -1.0));
        values.emplace_back(std::make_pair<double, double>(1.0, 0.0));
        values.emplace_back(std::make_pair<double, double>(1.5, 1.0));
        values.emplace_back(std::make_pair<double, double>(2.0, 3.0));

        curve = new curve_type();
        log << "Testing with value" << std::endl;
        log << test_curve_with_values(curve, values) << std::endl;
        delete curve;
        curve = new curve_type();
        log << "Testing with points" << std::endl;
        log << test_curve_with_points(curve, values) << std::endl;
        delete curve;

        values.clear();
        log << std::endl;


        log << "cycle points, -3.0, 0.0, 3.0, 0.0, -3.0" << std::endl;
        values.emplace_back(std::make_pair<double, double>(0.0, -3.0));
        values.emplace_back(std::make_pair<double, double>(0.5, 0.0));
        values.emplace_back(std::make_pair<double, double>(1.0, 3.0));
        values.emplace_back(std::make_pair<double, double>(1.5, 0.0));
        values.emplace_back(std::make_pair<double, double>(2.0, -3.0));

        curve = new curve_type();
        log << "Testing with value" << std::endl;
        log << test_curve_with_values(curve, values) << std::endl;
        delete curve;
        curve = new curve_type();
        log << "Testing with points" << std::endl;
        log << test_curve_with_points(curve, values) << std::endl;
        delete curve;

        values.clear();
        log << std::endl;


        log << "divers points, -3.0, -5.0, 0.0, 2.0, 1.0" << std::endl;
        values.emplace_back(std::make_pair<double, double>(0.0, -3.0));
        values.emplace_back(std::make_pair<double, double>(0.5, -5.0));
        values.emplace_back(std::make_pair<double, double>(1.0, 0.0));
        values.emplace_back(std::make_pair<double, double>(1.5, 2.0));
        values.emplace_back(std::make_pair<double, double>(2.0, 1.0));

        curve = new curve_type();
        log << "Testing with value" << std::endl;
        log << test_curve_with_values(curve, values) << std::endl;
        delete curve;
        curve = new curve_type();
        log << "Testing with points" << std::endl;
        log << test_curve_with_points(curve, values) << std::endl;
        delete curve;

        values.clear();
        log << std::endl;


        log << test_pose_handle_adding_points(bitbots_splines::PoseHandle(
                std::make_shared<curve_type>(), std::make_shared<curve_type>()
                ,std::make_shared<curve_type>(), std::make_shared<curve_type>()
                ,std::make_shared<curve_type>(), std::make_shared<curve_type>()));
        log << test_pose_handle_with_function(std::make_shared<bitbots_splines::PoseHandle>(
                std::make_shared<curve_type>(), std::make_shared<curve_type>()
                ,std::make_shared<curve_type>(), std::make_shared<curve_type>()
                ,std::make_shared<curve_type>(), std::make_shared<curve_type>()));

        bitbots_throw::SystemPublisher::print_to_file(log.str(), type_name + ".csv", false);
    }

    void TestingMovement::curve_with_radian(){
        std::shared_ptr<bitbots_splines::CubicSpline> pose = std::make_shared<bitbots_splines::CubicSpline>();
        SystemPublisher::publish_info("Start Testing", "curve_with_radian");

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
            pose->add_point(bitbots_splines::Curve::Point{it.first, it.second});
        }

        int point = 1;
        int successful = 0;
        for(auto it : test_values){
            double t = pose->position(it.first);
            if(it.second != t){
                SystemPublisher::publish_info("Check Failed (Point: " + std::to_string(point) + ", time: " + std::to_string(it.first) + ", position: " + std::to_string(it.second) + ")", "curve_with_radian");
            }else{
                ++successful;
            }

            ++point;
        }

        SystemPublisher::publish_info("Finished Testing, [" + std::to_string(successful) + " out of " + std::to_string(point) + " successful]", "curve_with_radian");
    }

    std::string
    TestingMovement::test_curve_with_values(bitbots_splines::Curve * curve, std::vector<std::pair<double, double>> values){
        std::stringstream log;

        for(auto it : values){
            curve->add_point(it.first, it.second);
        }
        log << curve->get_debug_csv() << std::endl;
        log << generate_points_csv(curve) << std::endl;

        return log.str();
    }

    std::string
    TestingMovement::test_curve_with_points(bitbots_splines::Curve * curve, std::vector<std::pair<double, double>> values){
        std::stringstream log;

        for(auto it : values){
            curve->add_point(bitbots_splines::Curve::Point{it.first, it.second, 0.0, 0.0});
        }
        log << curve->get_debug_csv() << std::endl;
        log << generate_points_csv(curve) << std::endl;

        return log.str();
    }

    std::string TestingMovement::test_pose_handle_adding_points(bitbots_splines::PoseHandle handle){
        std::stringstream log;
        log << "x-points, -3.0, -1.0, 0.0, 1.0, 3.0, (ascending)" << std::endl;
        log << "y-points, -3.0, 0.0, 3.0, 0.0, -3.0, (cycle)" << std::endl;
        log << "z-points, -3.0, -5.0, 0.0, 2.0, 1.0, (divers)" << std::endl;

        handle.x()->add_point(0.0, -3.0);
        handle.x()->add_point(0.5, -1.0);
        handle.x()->add_point(1.0, 0.0);
        handle.x()->add_point(1.5, 1.0);
        handle.x()->add_point(2.0, 3.0);

        handle.y()->add_point(0.0, -3.0);
        handle.y()->add_point(0.5, 0.0);
        handle.y()->add_point(1.0, 3.0);
        handle.y()->add_point(1.5, 0.0);
        handle.y()->add_point(2.0, -3.0);

        handle.z()->add_point(0.0, -3.0);
        handle.z()->add_point(0.5, -5.0);
        handle.z()->add_point(1.0, 0.0);
        handle.z()->add_point(1.5, 2.0);
        handle.z()->add_point(2.0, 1.0);

        log << handle.get_debug_csv() << std::endl;
        log << generate_points_csv(handle, 2.1) << std::endl;
        return log.str();
    }

    std::string TestingMovement::test_pose_handle_with_function(std::shared_ptr<bitbots_splines::PoseHandle> handle){
        std::stringstream log;
        Struct3dRPY zeros(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        log << "x-points, -3.0, -1.0, 0.0, 1.0, 3.0, (ascending)" << std::endl;
        log << "y-points, -3.0, 0.0, 3.0, 0.0, -3.0, (cycle)" << std::endl;
        log << "z-points, -3.0, -5.0, 0.0, 2.0, 1.0, (divers)" << std::endl;

        add_point(handle, 0.0, Struct3dRPY(-3.0, -3.0, -3.0, 0.0, 0.0, 0.0), zeros, zeros);
        add_point(handle, 0.5, Struct3dRPY(-1.0, 0.0, -5.0, 0.0, 0.0, 0.0), zeros, zeros);
        add_point(handle, 1.0, Struct3dRPY(0.0, 3.0, 0.0, 0.0, 0.0, 0.0), zeros, zeros);
        add_point(handle, 1.5, Struct3dRPY(1.0, 0.0, 2.0, 0.0, 0.0, 0.0), zeros, zeros);
        add_point(handle, 2.0, Struct3dRPY(3.0, -3.0, 1.0, 0.0, 0.0, 0.0), zeros, zeros);

        log << handle->get_debug_csv() << std::endl;
        log << generate_points_csv(*handle, 2.1) << std::endl;
        return log.str();
    }
}