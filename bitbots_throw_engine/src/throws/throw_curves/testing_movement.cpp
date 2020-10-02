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
    }

    double TestingMovement::init_material(){
        SystemPublisher::publish_info("Start Testing", "TestingMovement");
        trajectory_time_ = 0.0;

        curve_with_radian();
        test_basic_throw_movement();
        test_curves();
        move_arms_and_feet();
        squat();
        rotate_right_hand();

        SystemPublisher::publish_info("Finished Testing", "TestingMovement");
        return trajectory_time_;
    }

    void TestingMovement::test_curves(){
        //test_curve_type<bitbots_splines::Beziercurve>("Beziercurve");
        test_curve_type<bitbots_splines::LinearSpline>("LinearSpline");
        test_curve_type<bitbots_splines::CubicSpline>("CubicSpline");

        // TODO: Find out why point are not getting added to Smooth Spline
        // test_curve_type<bitbots_splines::SmoothSpline>("SmoothSpline");

        SystemPublisher::publish_info("Finished Testing", "test_curves");
    }

    void TestingMovement::move_arms_and_feet(){
        SystemPublisher::publish_info("Start Testing Movement III", "TestingMovement");
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        SystemPublisher::publish_info("Finished Testing Movement III", "TestingMovement");
    }

    void TestingMovement::squat(){
        SystemPublisher::publish_info("Start Testing Movement II", "TestingMovement");
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.6;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        SystemPublisher::publish_info("Finished Testing Movement II", "TestingMovement");
    }

    void TestingMovement::rotate_right_hand(){
        SystemPublisher::publish_info("Start Testing Movement I", "TestingMovement");
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {-3.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, 3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, -3.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;

        sp_material_->add_point_to_right_hand(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_hand(trajectory_time_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_right_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        sp_material_->add_point_to_left_foot(trajectory_time_, {0.0, 0.0, -3.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        trajectory_time_ += 0.3;
        SystemPublisher::publish_info("Finished Testing Movement I", "TestingMovement");
    }

    template<typename curve_type>
    void TestingMovement::test_curve_type(std::string const & type_name){
        std::stringstream log;
        bitbots_splines::Curve * curve;
        SystemPublisher system_publisher;
        std::vector<std::pair<double, double>> values;
        SystemPublisher::publish_info("Testing " + type_name, "test_curve");


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
        SystemPublisher::publish_info("Start Testing", "curve_with_radian");

        std::vector<bitbots_splines::Curve::Point> test_values;
        test_values.emplace_back(bitbots_splines::Curve::Point{0, 0, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{0.25, 0, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{0.5, 0, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{0.75, 5.49779, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{1, 5.49779, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{1.33333, 1.5708, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{1.66667, 1.5708, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{2, 2.35619, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{2.5, 1.5708, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{3, 5.75959, 0, 0});
        test_values.emplace_back(bitbots_splines::Curve::Point{4, 0, 0, 0});


        SystemPublisher::publish_info("Testing Spline Direct", "curve_with_radian");
        std::shared_ptr<bitbots_splines::CubicSpline> spline = std::make_shared<bitbots_splines::CubicSpline>();
        for(auto it : test_values){
            spline->add_point(it.time_, it.position_);
        }
        check_curve(spline, test_values);

        SystemPublisher::publish_info("Testing Spline via PoseHandle", "curve_with_radian");
        auto pose = create_pose_handle<bitbots_splines::CubicSpline>();
        for(auto it : test_values){
            pose->pitch()->add_point(it.time_, it.position_, 0.0, 0.0);
        }
        check_curve(pose->pitch(), test_values);

        SystemPublisher::publish_info("Testing Spline via PoseHandle with ThrowUtility", "curve_with_radian");
        auto pose_u = create_pose_handle<bitbots_splines::CubicSpline>();
        for(auto it : test_values){
            add_point(pose_u
                     ,it.time_
                     ,{0.0, 0.0, 0.0, 0.0, it.position_, 0.0}
                     ,{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
                     ,{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        }
        check_curve(pose_u->pitch(), test_values);
    }

    void TestingMovement::test_basic_throw_movement(){
        SystemPublisher::publish_info("Testing ThrowMaterial base class", "test_basic_throw_movement");
        ThrowMovement movement(std::make_shared<ThrowMaterial>(create_pose_handle<bitbots_splines::CubicSpline>()
                                                              ,create_pose_handle<bitbots_splines::CubicSpline>()
                                                              ,create_pose_handle<bitbots_splines::CubicSpline>()
                                                              ,create_pose_handle<bitbots_splines::CubicSpline>()));
        movement.init(sp_service_);
        check_movement(movement);
        SystemPublisher::publish_info("finished testing", "test_basic_throw_movement");
    }

    void TestingMovement::check_curve(std::shared_ptr<bitbots_splines::Curve> sp_curve, std::vector<bitbots_splines::Curve::Point> const & values){
        int point = 0;
        int successful = 0;
        for(auto it : values){
            ++point;
            double t = sp_curve->position(it.time_);
            double e = std::abs(it.position_ - t);
            if(e > 0.000001){
                SystemPublisher::publish_info("Check Failed (Point: " + std::to_string(point) + ", time: " + std::to_string(it.time_) + ", position: " + std::to_string(it.position_) + ", position-from-curve: " + std::to_string(t) + ")", "check_curve");
            }else{
                ++successful;
            }
        }
        SystemPublisher::publish_info("Finished Testing, [" + std::to_string(successful) + " out of " + std::to_string(point) + " successful]", "check_curve");
    }

    void TestingMovement::check_joint(std::shared_ptr<bitbots_splines::PoseHandle> sp_joint
                                      , const std::vector<std::vector<bitbots_splines::Curve::Point>> & joint_values){
        SystemPublisher::publish_info("========== check x ==========", "check_joint - joint_values");
        check_curve(sp_joint->x(), joint_values.at(0));
        SystemPublisher::publish_info("==========", "check_joint - curve");
        check_curve(sp_joint->x(), sp_joint->x()->points());

        SystemPublisher::publish_info("========== check y ==========", "check_joint - joint_values");
        check_curve(sp_joint->y(), joint_values.at(1));
        SystemPublisher::publish_info("==========", "check_joint - curve");
        check_curve(sp_joint->y(), sp_joint->y()->points());

        SystemPublisher::publish_info("========== check z ==========", "check_joint - joint_values");
        check_curve(sp_joint->z(), joint_values.at(2));
        SystemPublisher::publish_info("==========", "check_joint - curve");
        check_curve(sp_joint->z(), sp_joint->z()->points());

        SystemPublisher::publish_info("========== check roll ==========", "check_joint - joint_values");
        check_curve(sp_joint->roll(), joint_values.at(3));
        SystemPublisher::publish_info("==========", "check_joint - curve");
        check_curve(sp_joint->roll(), sp_joint->roll()->points());

        SystemPublisher::publish_info("========== check pitch ==========", "check_joint - joint_values");
        check_curve(sp_joint->pitch(), joint_values.at(4));
        SystemPublisher::publish_info("==========", "check_joint - curve");
        check_curve(sp_joint->pitch(), sp_joint->pitch()->points());

        SystemPublisher::publish_info("========== check yaw ==========", "check_joint - joint_values");
        check_curve(sp_joint->yaw(), joint_values.at(5));
        SystemPublisher::publish_info("==========", "check_joint - curve");
        check_curve(sp_joint->yaw(), sp_joint->yaw()->points());
    }

    void TestingMovement::check_movement(ThrowMovement & movement){
        auto material = movement.create_material(true);

        SystemPublisher::publish_info("==================== check left hand ====================", "check_movement");
        check_joint(material->get_left_hand(), movement.get_left_hand_points());

        SystemPublisher::publish_info("==================== check right hand ====================", "check_movement");
        check_joint(material->get_right_hand(), movement.get_right_hand_points());

        SystemPublisher::publish_info("==================== check left foot ====================", "check_movement");
        check_joint(material->get_left_foot(), movement.get_left_foot_points());

        SystemPublisher::publish_info("==================== check right foot ====================", "check_movement");
        check_joint(material->get_right_foot(), movement.get_right_foot_points());
    }

    std::string
    TestingMovement::test_curve_with_values(bitbots_splines::Curve * curve, std::vector<std::pair<double, double>> values){
        std::stringstream log;

        for(auto it : values){
            curve->add_point(it.first, it.second, 0.0, 0.0);
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

        handle.x()->add_point(0.0, -3.0, 0.0, 0.0);
        handle.x()->add_point(0.5, -1.0, 0.0, 0.0);
        handle.x()->add_point(1.0, 0.0, 0.0, 0.0);
        handle.x()->add_point(1.5, 1.0, 0.0, 0.0);
        handle.x()->add_point(2.0, 3.0, 0.0, 0.0);

        handle.y()->add_point(0.0, -3.0, 0.0, 0.0);
        handle.y()->add_point(0.5, 0.0, 0.0, 0.0);
        handle.y()->add_point(1.0, 3.0, 0.0, 0.0);
        handle.y()->add_point(1.5, 0.0, 0.0, 0.0);
        handle.y()->add_point(2.0, -3.0, 0.0, 0.0);

        handle.z()->add_point(0.0, -3.0, 0.0, 0.0);
        handle.z()->add_point(0.5, -5.0, 0.0, 0.0);
        handle.z()->add_point(1.0, 0.0, 0.0, 0.0);
        handle.z()->add_point(1.5, 2.0, 0.0, 0.0);
        handle.z()->add_point(2.0, 1.0, 0.0, 0.0);

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