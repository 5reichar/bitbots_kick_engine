#include "engine/throw_engine.h"
#include <math.h>

void ThrowEngine::set_throw_types(std::shared_ptr<ThrowTypeParameter> types)
{
	sp_throw_types_ = types;
}
void ThrowEngine::set_engine_parameter(std::shared_ptr<ThrowEngineParameter> parameter)
{
	sp_engine_parameter_ = parameter;
}

bool ThrowEngine::throw_ball(struct3d & ball_position, struct3d & target_position)
{
	//TODO: testing
	//TODO: cleanup

	bool throw_calc_succ = false;
	sp_current_throw_ = sp_throw_factory_->create_throw(sp_throw_types_, calculate_distace(target_position));
	if (sp_current_throw_)
	{
		auto throw_parameter = build_throw_parameter(ball_position, target_position);

		throw_calc_succ |= sp_current_throw_->calculate_trajectories(throw_parameter);
	}

	return throw_calc_succ;
}

std::shared_ptr<ThrowParameter> ThrowEngine::build_throw_parameter(struct3d & ball_position, struct3d & target_position)
{
	auto sp_parameter = ThrowParameterBuilder::build_from_dynamic_reconf(sp_engine_parameter_);

	sp_parameter->left_hand_start_position_ = ;
	sp_parameter->left_hand_end_position_ = ;
	sp_parameter->right_hand_start_position_ = ;
	sp_parameter->right_hand_end_position_ = ;
	sp_parameter->ball_position_ = ball_position;
	sp_parameter->throw_goal_position_ = target_position;
	sp_parameter->throw_velocity_ = ;
	sp_parameter->pick_up_orientation_ = ;
	sp_parameter->pick_up_bow_angle_ = ;
	sp_parameter->throw_orientation_ = ;
	sp_parameter->throw_start_pitch_ = ;
	sp_parameter->throw_release_pitch_ = ;

	return sp_parameter;
}