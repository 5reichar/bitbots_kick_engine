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
		auto throw_parameter = build_throw_parameter();

		throw_calc_succ |= sp_current_throw_->calculate_trajectories(throw_parameter);
	}

	return throw_calc_succ;
}

std::shared_ptr<ThrowParameter> ThrowEngine::build_throw_parameter()
{
	auto sp_parameter = ThrowParameterBuilder::build_from_dynamic_reconf(sp_engine_parameter_);

	return sp_parameter;
}


double ThrowEngine::calculate_distace(struct3d & point)
{
	return sqrt(pow(point.x, 2) + pow(point.y, 2));
}