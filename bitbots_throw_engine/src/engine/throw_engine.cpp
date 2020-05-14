#include "engine/throw_engine.h"
#include <math.h>
#include "parameter/throw_parameter_builder.h"

ThrowResponse ThrowEngine::update(double dt)
{
	//TODO: implement
	//TODO: testing
	//TODO: cleanup

	ThrowResponse response;

	response.support_foot_to_trunk_ = sp_current_throw_->get_pose_trunk()->get_tf_transform(time_);
	response.support_foot_to_left_hand_ = sp_current_throw_->get_pose_left_hand()->get_tf_transform(time_);
	response.support_foot_to_right_hand_ = sp_current_throw_->get_pose_right_hand()->get_tf_transform(time_);

	time_ += dt;
	return response;
}

void ThrowEngine::reset()
{
	//TODO: implement
	time_ = 0;

}

void ThrowEngine::set_goals(const ThrowRequest & request)
{
    // Wrapper to keep style consistence

	setGoals(request);
}

void ThrowEngine::setGoals(const ThrowRequest & request)
{
	//TODO: testing
	//TODO: cleanup

	auto throw_type = sp_throw_factory_->get_throw_type(sp_throw_types_, ThrowParameterBuilder::calculate_distace(request.goal_position_));
	sp_current_throw_ = sp_throw_factory_->create_throw(throw_type);
	auto throw_parameter = ThrowParameterBuilder::build_from_dynamic_reconf(sp_engine_parameter_, throw_type, request.ball_position_, request.goal_position_);
	sp_current_throw_->calculate_trajectories(throw_parameter);
}

int ThrowEngine::get_percent_done() const
{
    // Wrapper to keep style consistence

	return getPercentDone();
}

int ThrowEngine::getPercentDone() const
{
	//TODO: implement

	return 0;
}

void ThrowEngine::set_throw_types(std::shared_ptr<ThrowTypeParameter> types)
{
	sp_throw_types_ = types;
}
void ThrowEngine::set_engine_parameter(std::shared_ptr<ThrowEngineParameter> parameter)
{
	sp_engine_parameter_ = parameter;
}