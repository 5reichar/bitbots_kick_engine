#ifndef THROW_PARAMETER_BUILDER_H
#define THROW_PARAMETER_BUILDER_H

#include <math.h>
#include <memory>
#include "parameter/throw_parameter.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"
#include "ros_interface/publisher/system_publisher.h"
#include "utility/throw_utilities.h"

class ThrowParameterBuilder
{
public:
    static std::shared_ptr<ThrowParameter> build_from_dynamic_reconf(std::shared_ptr<ThrowEngineParameter> & engine_parameter, std::shared_ptr<ThrowType> & throw_type, Struct3d ball_position, Struct3d throw_goal_position)
    {
        auto sp_parameter = build_default();

        auto left_hand_position = -0.5 * engine_parameter->hand_distance_;
        auto right_hand_position = 0.5 * engine_parameter->hand_distance_;
        auto throw_orientation_angle = calculate_angle(throw_goal_position);
        auto velocity = calculate_velocity(engine_parameter, throw_goal_position);

        sp_parameter->start_left_hand_position_.x_ = left_hand_position;
        sp_parameter->start_left_hand_position_.y_ = 0.0;
        sp_parameter->start_left_hand_position_.z_ = 0.0;

        sp_parameter->start_right_hand_position_.x_ = right_hand_position;
        sp_parameter->start_right_hand_position_.y_ = 0.0;
        sp_parameter->start_right_hand_position_.z_ = 0.0;

        sp_parameter->pick_up_left_hand_position_.x_ = ball_position.x_ - engine_parameter->ball_radius_;
        sp_parameter->pick_up_left_hand_position_.y_ = ball_position.y_;
        sp_parameter->pick_up_left_hand_position_.z_ = ball_position.z_;

        sp_parameter->pick_up_right_hand_position_.x_ = ball_position.x_ + engine_parameter->ball_radius_;
        sp_parameter->pick_up_right_hand_position_.y_ = ball_position.y_;
        sp_parameter->pick_up_right_hand_position_.z_ = ball_position.z_;

        sp_parameter->pick_up_left_hand_axis_.roll_ = 0.0;
        sp_parameter->pick_up_left_hand_axis_.pitch_ = 0.0;
        sp_parameter->pick_up_left_hand_axis_.yaw_ = 0.0;

        sp_parameter->pick_up_right_hand_axis_.roll_ = 0.0;
        sp_parameter->pick_up_right_hand_axis_.pitch_ = 0.0;
        sp_parameter->pick_up_right_hand_axis_.yaw_ = 0.0;

        sp_parameter->pick_up_trunk_axis_.roll_ = 0.0;
        sp_parameter->pick_up_trunk_axis_.pitch_ = 0.0;
        sp_parameter->pick_up_trunk_axis_.yaw_ = calculate_angle(ball_position);

        sp_parameter->throw_start_left_hand_position_.x_ = engine_parameter->throw_start_position_.x_ - engine_parameter->ball_radius_;
        sp_parameter->throw_start_left_hand_position_.y_ = engine_parameter->throw_start_position_.y_;
        sp_parameter->throw_start_left_hand_position_.z_ = engine_parameter->throw_start_position_.z_;

        sp_parameter->throw_start_right_hand_position_.x_ = engine_parameter->throw_start_position_.x_ + engine_parameter->ball_radius_;
        sp_parameter->throw_start_right_hand_position_.y_ = engine_parameter->throw_start_position_.y_;
        sp_parameter->throw_start_right_hand_position_.z_ = engine_parameter->throw_start_position_.z_;

        sp_parameter->throw_start_left_hand_axis_.roll_ = 0.0;
        sp_parameter->throw_start_left_hand_axis_.pitch_ = 0.0;
        sp_parameter->throw_start_left_hand_axis_.yaw_ = 0.0;

        sp_parameter->throw_start_right_hand_axis_.roll_ = 0.0;
        sp_parameter->throw_start_right_hand_axis_.pitch_ = 0.0;
        sp_parameter->throw_start_right_hand_axis_.yaw_ = 0.0;

        sp_parameter->throw_start_trunk_axis_.roll_ = 0.0;
        sp_parameter->throw_start_trunk_axis_.pitch_ = 0.0;
        sp_parameter->throw_start_trunk_axis_.yaw_ = throw_orientation_angle;

        sp_parameter->throw_release_left_hand_position_.x_ = engine_parameter->throw_release_position_.x_ - engine_parameter->ball_radius_;
        sp_parameter->throw_release_left_hand_position_.y_ = engine_parameter->throw_release_position_.y_;
        sp_parameter->throw_release_left_hand_position_.z_ = engine_parameter->throw_release_position_.z_;

        sp_parameter->throw_release_right_hand_position_.x_ = engine_parameter->throw_release_position_.x_ + engine_parameter->ball_radius_;
        sp_parameter->throw_release_right_hand_position_.y_ = engine_parameter->throw_release_position_.y_;
        sp_parameter->throw_release_right_hand_position_.z_ = engine_parameter->throw_release_position_.z_;

        sp_parameter->throw_release_left_hand_axis_.roll_ = 0.0;
        sp_parameter->throw_release_left_hand_axis_.pitch_ = 0.0;
        sp_parameter->throw_release_left_hand_axis_.yaw_ = -90.0;

        sp_parameter->throw_release_right_hand_axis_.roll_ = 0.0;
        sp_parameter->throw_release_right_hand_axis_.pitch_ = 0.0;
        sp_parameter->throw_release_right_hand_axis_.yaw_ = 90.0;

        sp_parameter->throw_release_trunk_axis_.roll_ = 0.0;
        sp_parameter->throw_release_trunk_axis_.pitch_ = 0.0;
        sp_parameter->throw_release_trunk_axis_.yaw_ = throw_orientation_angle;

        sp_parameter->throw_velocity_.x_ = sin(throw_orientation_angle) * velocity;
        sp_parameter->throw_velocity_.y_ = cos(throw_orientation_angle) * velocity;
        sp_parameter->throw_velocity_.z_ = 0.0;

        sp_parameter->end_left_hand_position_.x_ = left_hand_position;
        sp_parameter->end_left_hand_position_.y_ = 0.0;
        sp_parameter->end_left_hand_position_.z_ = 0.0;

        sp_parameter->end_right_hand_position_.x_ = right_hand_position;
        sp_parameter->end_right_hand_position_.y_ = 0.0;
        sp_parameter->end_right_hand_position_.z_ = 0.0;

        sp_parameter->throw_goal_position_ = throw_goal_position;
        
    	sp_parameter->movement_cycle_frequence_ = engine_parameter->frequency_;

        sp_parameter->pick_up_duration_share_ = throw_type->pick_up_duration_share_ == 0 ? engine_parameter->pick_up_duration_share_ : throw_type->pick_up_duration_share_;
        sp_parameter->throw_preparation_duration_share_ = throw_type->throw_preparation_duration_share_ == 0 ? engine_parameter->throw_preparation_duration_share_ : throw_type->throw_preparation_duration_share_;
        sp_parameter->throw_duration_share_ = throw_type->throw_duration_share_ == 0 ? engine_parameter->throw_duration_share_ : throw_type->throw_duration_share_;
        sp_parameter->throw_conclusion_duration_share_ = throw_type->throw_conclusion_duration_share_ == 0 ? engine_parameter->throw_conclusion_duration_share_ : throw_type->throw_conclusion_duration_share_;
        check_duration_share(sp_parameter, engine_parameter);

        sp_parameter->throw_anlge_ = throw_type->throw_anlge_ == 0 ? engine_parameter->throw_anlge_ : throw_type->throw_anlge_;


        return sp_parameter;
    };

    static std::shared_ptr<ThrowParameter> build_default()
    {
        // TODO: enter better default values
        return std::make_shared<ThrowParameter>(Struct3d{0.0, 0.0, 0.0},  //start_left_hand_position
                                                Struct3d{0.0, 0.0, 0.0},  //start_right_hand_position
                                                Struct3d{0.0, 0.0, 0.0},  //pick_up_left_hand_position
                                                Struct3d{0.0, 0.0, 0.0},  //pick_up_right_hand_position
                                                Struct3dRPY{0.0, 0.0, 0.0},  //pick_up_left_hand_axis
                                                Struct3dRPY{0.0, 0.0, 0.0},  //pick_up_right_hand_axis
                                                Struct3dRPY{0.0, 0.0, 0.0},  //pick_up_trunk_axis
                                                Struct3d{0.0, 0.0, 0.0},  //throw_start_left_hand_position
                                                Struct3d{0.0, 0.0, 0.0},  //throw_start_right_hand_position
                                                Struct3dRPY{0.0, 0.0, 0.0},  //throw_start_left_hand_axis
                                                Struct3dRPY{0.0, 0.0, 0.0},  //throw_start_right_hand_axis
                                                Struct3dRPY{0.0, 0.0, 0.0},  //throw_start_trunk_axis
                                                Struct3d{0.0, 0.0, 0.0},  //throw_release_left_hand_position
                                                Struct3d{0.0, 0.0, 0.0},  //throw_release_right_hand_position
                                                Struct3dRPY{0.0, 0.0, 0.0},  //throw_release_left_hand_axis
                                                Struct3dRPY{0.0, 0.0, 0.0},  //throw_release_right_hand_axis
                                                Struct3dRPY{0.0, 0.0, 0.0},  //throw_release_trunk_axis
                                                Struct3d{0.0, 0.0, 0.0},  //throw_velocity
                                                Struct3d{0.0, 0.0, 0.0},  //end_left_hand_position
                                                Struct3d{0.0, 0.0, 0.0},  //end_right_hand_position
                                                Struct3d{0.0, 0.0, 0.0},  //throw_goal_position
                                                0.0,  //movement_cycle_frequence
                                                0.25,  //pick_up_duration_share
                                                0.25,  //throw_preparation_duration_share
                                                0.25,  //throw_duration_share
                                                0.25,  //throw_conclusion_duration_share
                                                30.0);  //throw_anlge
    };

protected:
    static double calculate_velocity(std::shared_ptr<ThrowEngineParameter> & engine_parameter, Struct3d throw_goal_position)
    {
        auto time = engine_parameter->throw_release_position_.z_ / engine_parameter->gravity_;
        return calculate_distace(throw_goal_position) / time;
    }

    static double calculate_angle(Struct3d & point)
    {
        return atan((point.y_ / point.x_));
    }

    static void check_duration_share(std::shared_ptr<ThrowParameter> & sp_parameter, std::shared_ptr<ThrowEngineParameter> & engine_parameter)
    {
        double total_share_div = sp_parameter->pick_up_duration_share_ + sp_parameter->throw_preparation_duration_share_ + sp_parameter->throw_duration_share_ + sp_parameter->throw_conclusion_duration_share_ - 1;
        if (total_share_div > 0.0000001f) // check if total share is bigger than 1
        {
            // Reduce the shares equality to remain the proportion of the shares 
            sp_parameter->pick_up_duration_share_ -= total_share_div / 4;
            sp_parameter->throw_preparation_duration_share_ -= total_share_div / 4;
            sp_parameter->throw_duration_share_ -= total_share_div / 4;
            sp_parameter->throw_conclusion_duration_share_ -= total_share_div / 4;

            SystemPublisher::publish_warning("The Shares of the throw movement steps are reduced, becase the shares added up to a bigger value than 1");
        }
    }
};

#endif