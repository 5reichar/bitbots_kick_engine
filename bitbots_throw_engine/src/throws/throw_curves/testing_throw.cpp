//
// Created by dmraw on 09.07.20.
//

#include "throws/throw_curves/testing_throw.h"
#include "../../bitbots_splines_extension/include/spline/linear_spline.h"

namespace bitbots_throw{

    TestingThrow::TestingThrow()
            :ThrowCurve(
            std::make_shared<bitbots_splines::PoseHandle>( // Left Hand
                    std::make_shared<bitbots_splines::LinearSpline>() // x
                    ,std::make_shared<bitbots_splines::LinearSpline>() // y
                    ,std::make_shared<bitbots_splines::LinearSpline>() // z
                    ,std::make_shared<bitbots_splines::LinearSpline>() // roll
                    ,std::make_shared<bitbots_splines::LinearSpline>() // pitch
                    ,std::make_shared<bitbots_splines::LinearSpline>() // yaw
            ),
            std::make_shared<bitbots_splines::PoseHandle>( // Right Hand
                    std::make_shared<bitbots_splines::LinearSpline>() // x
                    ,std::make_shared<bitbots_splines::LinearSpline>() // y
                    ,std::make_shared<bitbots_splines::LinearSpline>() // z
                    ,std::make_shared<bitbots_splines::LinearSpline>() // roll
                    ,std::make_shared<bitbots_splines::LinearSpline>() // pitch
                    ,std::make_shared<bitbots_splines::LinearSpline>() // yaw
            ),
            std::make_shared<bitbots_splines::PoseHandle>( // Trunk
                    std::make_shared<bitbots_splines::LinearSpline>() // x
                    ,std::make_shared<bitbots_splines::LinearSpline>() // y
                    ,std::make_shared<bitbots_splines::LinearSpline>() // z
                    ,std::make_shared<bitbots_splines::LinearSpline>() // roll
                    ,std::make_shared<bitbots_splines::LinearSpline>() // pitch
                    ,std::make_shared<bitbots_splines::LinearSpline>() // yaw
            ),
            std::make_shared<bitbots_splines::PoseHandle>( // Left Feet
                    std::make_shared<bitbots_splines::LinearSpline>() // x
                    ,std::make_shared<bitbots_splines::LinearSpline>() // y
                    ,std::make_shared<bitbots_splines::LinearSpline>() // z
                    ,std::make_shared<bitbots_splines::LinearSpline>() // roll
                    ,std::make_shared<bitbots_splines::LinearSpline>() // pitch
                    ,std::make_shared<bitbots_splines::LinearSpline>() // yaw
            ),
            std::make_shared<bitbots_splines::PoseHandle>( // Right Feet
                    std::make_shared<bitbots_splines::LinearSpline>() // x
                    ,std::make_shared<bitbots_splines::LinearSpline>() // y
                    ,std::make_shared<bitbots_splines::LinearSpline>() // z
                    ,std::make_shared<bitbots_splines::LinearSpline>() // roll
                    ,std::make_shared<bitbots_splines::LinearSpline>() // pitch
                    ,std::make_shared<bitbots_splines::LinearSpline>() // yaw
            )){
        max_prepare_rounds_ = 20;
    }

    void TestingThrow::calculate_pick_up_ball_movement(double & time
                                                      ,std::shared_ptr<ThrowParameter> & throw_parameter){
        auto alternativ_value = Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        for(int round = 0; round < max_prepare_rounds_; ++round){
            add_points(sp_pose_left_feet_
                      ,round * throw_parameter->movement_cycle_frequency_
                      ,round%2 == 0 ? throw_parameter->start_left_feet_ : alternativ_value);
            add_points(sp_pose_right_feet_
                      ,round * throw_parameter->movement_cycle_frequency_
                      ,round%2 == 0 ? throw_parameter->start_right_feet_ : alternativ_value);
            add_points(sp_pose_left_hand_
                      ,round * throw_parameter->movement_cycle_frequency_
                      ,round%2 == 0 ? throw_parameter->start_left_arm_ : alternativ_value);
            add_points(sp_pose_right_hand_
                      ,round * throw_parameter->movement_cycle_frequency_
                      ,round%2 == 0 ? throw_parameter->start_left_arm_ : alternativ_value);

            time += throw_parameter->movement_cycle_frequency_;
        }
    }

    void TestingThrow::calculate_throw_preparation_movement(double & time
                                                           ,std::shared_ptr<ThrowParameter> & throw_parameter){
    }

    void TestingThrow::calculate_throw_movement(double & time
                                               ,std::shared_ptr<ThrowParameter> & throw_parameter){
    }

    void TestingThrow::calculate_throw_conclusion_movement(double & time
                                                          ,std::shared_ptr<ThrowParameter> & throw_parameter){
    }
}