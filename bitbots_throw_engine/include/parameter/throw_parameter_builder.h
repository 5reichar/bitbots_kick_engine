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
    static std::shared_ptr<ThrowParameter> build_from_dynamic_reconf(std::shared_ptr<ThrowEngineParameter> & engine_parameter, std::shared_ptr<ThrowTypeParameter> & throw_type, struct3d & ball_position, struct3d & throw_goal_position)
    {
        std::shared_ptr<ThrowParameter> sp_parameter;

    	sp_parameter->movement_cycle_frequence_ = engine_parameter->frequency_;
        sp_parameter->throw_start_position_ = engine_parameter->throw_start_position_;
        sp_parameter->throw_release_position_ = engine_parameter->throw_release_position_;
        
        sp_parameter->pick_up_duration_share_ = engine_parameter->pick_up_duration_share_;
        sp_parameter->throw_preparation_duration_share_ = engine_parameter->throw_preparation_duration_share_;
        sp_parameter->throw_duration_share_ = engine_parameter->throw_duration_share_;

        sp_parameter->ball_position_ = ball_position;
        sp_parameter->throw_goal_position_ = throw_goal_position;

        sp_parameter->throw_distance_ = calculate_distace(throw_goal_position);
        auto time = sp_parameter->throw_release_position_.z_ / engine_parameter->gravity_;
        sp_parameter->throw_velocity_ = sp_parameter->throw_distance_ / time;

        sp_parameter->pick_up_orientation_ = calculate_angle(ball_position);
        sp_parameter->throw_orientation_ = calculate_angle(throw_goal_position);

        sp_parameter->left_hand_start_position_ = 0.5 * engine_parameter->hand_distance_;
        sp_parameter->right_hand_start_position_ = -0.5 * engine_parameter->hand_distance_;

        sp_parameter->left_hand_end_position_ = 0.5 * engine_parameter->hand_distance_;
        sp_parameter->right_hand_end_position_ = -0.5 * engine_parameter->hand_distance_;

        sp_parameter->pick_up_bow_angle_ = ;
        sp_parameter->throw_start_pitch_ = ;
        sp_parameter->throw_release_pitch_ = ;

        return sp_parameter;
    };

    double calculate_distace(struct3d & point)
    {
        return sqrt(pow(point.x_, 2) + pow(point.y_, 2));
    }

    double calculate_angle(struct3d & point)
    {
        return atan((point.y_ / point.x_));
    }

protected:

};

#endif