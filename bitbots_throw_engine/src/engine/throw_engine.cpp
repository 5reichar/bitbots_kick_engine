#include "engine/throw_engine.h"
#include <math.h>
#include "parameter/throw_parameter_builder.h"

void ThrowEngine::set_throw_types(std::shared_ptr<ThrowTypeParameter> types)
{
	sp_throw_types_ = types;
}
void ThrowEngine::set_engine_parameter(std::shared_ptr<ThrowEngineParameter> parameter)
{
	sp_engine_parameter_ = parameter;
}

std::shared_ptr<ThrowCurve> ThrowEngine::get_current_throw() const
{
	return sp_current_throw_;
}

bool ThrowEngine::throw_ball(Struct3d & ball_position, Struct3d & target_position)
{
	//TODO: testing
	//TODO: cleanup

	auto throw_type = sp_throw_factory_->get_throw_type(sp_throw_types_, ThrowParameterBuilder::calculate_distace(target_position));
	sp_current_throw_ = sp_throw_factory_->create_throw(throw_type);
	auto throw_parameter = ThrowParameterBuilder::build_from_dynamic_reconf(sp_engine_parameter_, throw_type, ball_position, target_position);
	return sp_current_throw_ && sp_current_throw_->calculate_trajectories(throw_parameter);
}