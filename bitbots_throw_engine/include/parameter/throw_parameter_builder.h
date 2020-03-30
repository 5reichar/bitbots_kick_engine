#ifndef THROW_PARAMETER_BUILDER_H
#define THROW_PARAMETER_BUILDER_H

#include <math.h>
#include <memory>
#include "parameter/throw_parameter.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"

class ThrowParameterBuilder
{
public:
    static std::shared_ptr<ThrowParameter> build_from_dynamic_reconf(std::shared_ptr<ThrowEngineParameter> & engine_parameter, std::shared_ptr<ThrowType> & throw_type, struct3d & ball_position, struct3d & throw_goal_position)
    {
        std::shared_ptr<ThrowParameter> sp_parameter;

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

        sp_parameter->pick_up_left_hand_axis_.x_ = 0.0;
        sp_parameter->pick_up_left_hand_axis_.y_ = 0.0;
        sp_parameter->pick_up_left_hand_axis_.z_ = 0.0;

        sp_parameter->pick_up_right_hand_axis_.x_ = 0.0;
        sp_parameter->pick_up_right_hand_axis_.y_ = 0.0;
        sp_parameter->pick_up_right_hand_axis_.z_ = 0.0;

        sp_parameter->pick_up_trunk_axis_.x_ = 0.0;
        sp_parameter->pick_up_trunk_axis_.y_ = 0.0;
        sp_parameter->pick_up_trunk_axis_.z_ = calculate_angle(ball_position);

        sp_parameter->throw_start_left_hand_position_.x_ = engine_parameter->throw_start_position_.x_ - engine_parameter->ball_radius_;
        sp_parameter->throw_start_left_hand_position_.y_ = engine_parameter->throw_start_position_.y_;
        sp_parameter->throw_start_left_hand_position_.z_ = engine_parameter->throw_start_position_.z_;

        sp_parameter->throw_start_right_hand_position_.x_ = engine_parameter->throw_start_position_.x_ + engine_parameter->ball_radius_;
        sp_parameter->throw_start_right_hand_position_.y_ = engine_parameter->throw_start_position_.y_;
        sp_parameter->throw_start_right_hand_position_.z_ = engine_parameter->throw_start_position_.z_;

        sp_parameter->throw_start_left_hand_axis_.x_ = 0.0;
        sp_parameter->throw_start_left_hand_axis_.y_ = 0.0;
        sp_parameter->throw_start_left_hand_axis_.z_ = 0.0;

        sp_parameter->throw_start_right_hand_axis_.x_ = 0.0;
        sp_parameter->throw_start_right_hand_axis_.y_ = 0.0;
        sp_parameter->throw_start_right_hand_axis_.z_ = 0.0;

        sp_parameter->throw_start_trunk_axis_.x_ = 0.0;
        sp_parameter->throw_start_trunk_axis_.y_ = 0.0;
        sp_parameter->throw_start_trunk_axis_.z_ = throw_orientation_angle;

        sp_parameter->throw_release_left_hand_position_.x_ = engine_parameter->throw_release_position_.x_ - engine_parameter->ball_radius_;
        sp_parameter->throw_release_left_hand_position_.y_ = engine_parameter->throw_release_position_.y_;
        sp_parameter->throw_release_left_hand_position_.z_ = engine_parameter->throw_release_position_.z_;

        sp_parameter->throw_release_right_hand_position_.x_ = engine_parameter->throw_release_position_.x_ + engine_parameter->ball_radius_;
        sp_parameter->throw_release_right_hand_position_.y_ = engine_parameter->throw_release_position_.y_;
        sp_parameter->throw_release_right_hand_position_.z_ = engine_parameter->throw_release_position_.z_;

        sp_parameter->throw_release_left_hand_axis_.x_ = 0.0;
        sp_parameter->throw_release_left_hand_axis_.y_ = 0.0;
        sp_parameter->throw_release_left_hand_axis_.z_ = 0.0;

        sp_parameter->throw_release_right_hand_axis_.x_ = 0.0;
        sp_parameter->throw_release_right_hand_axis_.y_ = 0.0;
        sp_parameter->throw_release_right_hand_axis_.z_ = 0.0;

        sp_parameter->throw_release_trunk_axis_.x_ = 0.0;
        sp_parameter->throw_release_trunk_axis_.y_ = 0.0;
        sp_parameter->throw_release_trunk_axis_.z_ = throw_orientation_angle;

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

        if (throw_type->override_movement_shares_)
        {
            sp_parameter->pick_up_duration_share_ = throw_type->pick_up_duration_share_;
            sp_parameter->throw_preparation_duration_share_ = throw_type->throw_preparation_duration_share_;
            sp_parameter->throw_duration_share_ = throw_type->throw_duration_share_;
        }
        else
        {
            sp_parameter->pick_up_duration_share_ = engine_parameter->pick_up_duration_share_;
            sp_parameter->throw_preparation_duration_share_ = engine_parameter->throw_preparation_duration_share_;
            sp_parameter->throw_duration_share_ = engine_parameter->throw_duration_share_;
        }

        return sp_parameter;
    };

    static double calculate_distace(struct3d & point)
    {
        return sqrt(pow(point.x_, 2) + pow(point.y_, 2));
    }

protected:
    static double calculate_velocity(std::shared_ptr<ThrowEngineParameter> & engine_parameter, struct3d & throw_goal_position)
    {
        auto time = engine_parameter->throw_release_position_.z_ / engine_parameter->gravity_;
        return calculate_distace(throw_goal_position) / time;
    }

    static double calculate_angle(struct3d & point)
    {
        return atan((point.y_ / point.x_));
    }
};

#endif