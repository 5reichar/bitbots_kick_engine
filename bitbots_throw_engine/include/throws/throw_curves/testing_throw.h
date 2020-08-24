#ifndef BITBOTS_THROW_TESTING_THROW_H
#define BITBOTS_THROW_TESTING_THROW_H

#include "throw_curve.h"

namespace bitbots_throw{
    class TestingThrow : public ThrowCurve{
    public:
        TestingThrow();

    protected:
        void calculate_pick_up_ball_movement(std::shared_ptr<ThrowParameter> & throw_parameter) override;
        void calculate_throw_preparation_movement(std::shared_ptr<ThrowParameter> & throw_parameter) override;
        void calculate_throw_movement(std::shared_ptr<ThrowParameter> & throw_parameter) override ;
        void calculate_throw_conclusion_movement(std::shared_ptr<ThrowParameter> & throw_parameter) override;

        template<typename curve_type> void test_curve_type(std::string const & type_name);

        std::string test_curve_with_values(bitbots_splines::Curve * curve, std::vector<std::pair<double, double>> values);
        std::string test_curve_with_points(bitbots_splines::Curve * curve, std::vector<std::pair<double, double>> values);

        std::string test_pose_handle_adding_points(bitbots_splines::PoseHandle handle);
        std::string test_pose_handle_with_function(std::shared_ptr<bitbots_splines::PoseHandle> handle);

    private:
        int max_prepare_rounds_;
    };

}

#endif //BITBOTS_THROW_TESTING_THROW_H
