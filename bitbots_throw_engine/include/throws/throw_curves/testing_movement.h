#ifndef BITBOTS_THROW_TESTING_MOVEMENT_H
#define BITBOTS_THROW_TESTING_MOVEMENT_H

#include "throw_movement.h"

namespace bitbots_throw{
    class TestingMovement : public ThrowMovement{
    public:
        TestingMovement(std::shared_ptr<ThrowMaterial> material);

    private:
        double init_material() override;
        void test_curves();
        void move_arms_and_feet();
        void squat() ;
        void rotate_right_hand();

        template<typename curve_type> void test_curve_type(std::string const & type_name);

        void curve_with_radian();
        void test_basic_throw_movement();

        void check_curve(std::shared_ptr<bitbots_splines::Curve> sp_curve, const std::vector<bitbots_splines::Curve::Point> & values);
        void check_joint(std::shared_ptr<bitbots_splines::PoseHandle> sp_joint, const std::vector<std::vector<bitbots_splines::Curve::Point>> & joint_values);
        void check_movement(ThrowMovement & movement);

        std::string test_curve_with_values(bitbots_splines::Curve * curve, std::vector<std::pair<double, double>> values);
        std::string test_curve_with_points(bitbots_splines::Curve * curve, std::vector<std::pair<double, double>> values);
        std::string test_pose_handle_adding_points(bitbots_splines::PoseHandle handle);
        std::string test_pose_handle_with_function(std::shared_ptr<bitbots_splines::PoseHandle> handle);
    };

}

#endif //BITBOTS_THROW_TESTING_MOVEMENT_H
