#include "engine/throw_engine.h"

#include <math.h>

#include <utility>
#include "parameter/throw_parameter_builder.h"

namespace bitbots_throw{
	ThrowEngine::ThrowEngine()
					:sp_engine_parameter_(nullptr)
					,sp_throw_types_(nullptr){
		reset();
	}

	ThrowResponse ThrowEngine::update(double dt){
		ThrowResponse response;

		response.support_foot_to_trunk_ = sp_current_throw_->get_pose_trunk()->get_tf_transform(time_);
		response.support_foot_to_left_hand_ = sp_current_throw_->get_pose_left_hand()->get_tf_transform(time_);
		response.support_foot_to_right_hand_ = sp_current_throw_->get_pose_right_hand()->get_tf_transform(time_);
		response.support_foot_to_left_foot_ = sp_current_throw_->get_pose_left_feet()->get_tf_transform(time_);
		response.support_foot_to_right_foot_ = sp_current_throw_->get_pose_right_feet()->get_tf_transform(time_);

		time_ += dt;
		return response;
	}

	void ThrowEngine::reset(){
		time_ = 0.0;
		throw_duration_ = 0.0;
		sp_current_throw_ = nullptr;
	}

	int ThrowEngine::get_percent_done() const{
		// Wrapper to keep style consistence
		return getPercentDone();
	}

	int ThrowEngine::getPercentDone() const{
	    return 0.0 == throw_duration_ ? 0 : (int) std::round(100.0 * (time_ / throw_duration_));
	}

	void ThrowEngine::set_goals(const ThrowRequest & request){
		// Wrapper to keep style consistence
		setGoals(request);
	}

	void ThrowEngine::setGoals(const ThrowRequest & request){
		auto throw_type_id = sp_throw_factory_->get_throw_type(sp_throw_types_
		                                                      ,request.goal_position_);
		sp_current_throw_ = sp_throw_factory_->create_throw(throw_type_id);
		auto throw_parameter = create_throw_parameter(throw_type_id, request);
		throw_duration_ = sp_current_throw_->calculate_trajectories(throw_parameter);
	}


	void ThrowEngine::set_throw_types(std::shared_ptr<ThrowTypeParameter> types){
		sp_throw_types_ = std::move(types);
	}

	void ThrowEngine::set_engine_parameter(std::shared_ptr<ThrowEngineParameter> parameter){
		sp_engine_parameter_ = std::move(parameter);
	}

	std::shared_ptr<ThrowParameter> ThrowEngine::create_throw_parameter(const ThrowTypeId throw_type_id
	                                                                   ,const ThrowRequest & request){
		return ThrowParameterBuilder::build_from_dynamic_reconf(sp_engine_parameter_
		                                                       ,sp_throw_types_->map_throw_types_[throw_type_id]
		                                                       ,request);
	}
}