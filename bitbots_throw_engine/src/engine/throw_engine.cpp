#include "engine/throw_engine.h"

#include <math.h>

#include <utility>
#include "parameter/throw_parameter_builder.h"
#include "parameter/throw_type_parameter_builder.h"

namespace bitbots_throw{
	ThrowEngine::ThrowEngine()
					:sp_engine_parameter_(nullptr)
					,sp_throw_types_(nullptr)
					,sp_throw_factory_(new ThrowFactory()){
		reset();
	}

	ThrowResponse ThrowEngine::update(double dt){
		ThrowResponse response;

		response.support_foot_to_trunk_ = sp_current_throw_->get_trunk_transform(time_);
		response.support_foot_to_left_hand_ = sp_current_throw_->get_left_hand_transform(time_);
		response.support_foot_to_right_hand_ = sp_current_throw_->get_right_hand_transform(time_);
		response.support_foot_to_left_foot_ = sp_current_throw_->get_left_feet_transform(time_);
		response.support_foot_to_right_foot_ = sp_current_throw_->get_right_feet_transform(time_);

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
		auto throw_type_id = sp_throw_factory_->get_throw_type(request.goal_position_);
		sp_current_throw_ = sp_throw_factory_->create_throw(throw_type_id);
		auto throw_parameter = create_throw_parameter(throw_type_id, request);
		throw_duration_ = sp_current_throw_->calculate_trajectories(throw_parameter);
		movement_stages_ = sp_current_throw_->get_movement_stage();
	}


	void ThrowEngine::set_throw_types(std::shared_ptr<ThrowTypeParameter> & types){
	    sp_throw_types_ = types;
	    sp_throw_factory_->set_throw_type_parameter(types);
	}

	void ThrowEngine::set_engine_parameter(std::shared_ptr<ThrowEngineParameter> parameter){
		sp_engine_parameter_ = std::move(parameter);
	}

	std::shared_ptr<ThrowParameter> ThrowEngine::create_throw_parameter(const ThrowTypeId throw_type_id
	                                                                   ,const ThrowRequest & request){
	    std::shared_ptr<ThrowType> type;

		if (sp_throw_types_->map_throw_types_.cend() == sp_throw_types_->map_throw_types_.find(throw_type_id)){
            type = sp_engine_parameter_->default_throw_;
		}else{
            type = sp_throw_types_->map_throw_types_[throw_type_id];

            if(type->throw_angle_ == 0){
                type->throw_angle_ = sp_engine_parameter_->default_throw_->throw_angle_;
            };

            if(type->throw_strength_ == 0){
                type->throw_strength_ = sp_engine_parameter_->default_throw_->throw_strength_;
            };

            if(type->movement_duration_ == 0){
                type->movement_duration_ = sp_engine_parameter_->default_throw_->movement_duration_;
            };

            if(type->movement_share_pick_up_ == 0){
                type->movement_share_pick_up_ = sp_engine_parameter_->default_throw_->movement_share_pick_up_;
            };

            if(type->movement_share_preparation_ == 0){
                type->movement_share_preparation_ = sp_engine_parameter_->default_throw_->movement_share_preparation_;
            };

            if(type->movement_share_throw_ == 0){
                type->movement_share_throw_ = sp_engine_parameter_->default_throw_->movement_share_throw_;
            };

            if(type->movement_share_conclusion_ == 0){
                type->movement_share_conclusion_ = sp_engine_parameter_->default_throw_->movement_share_conclusion_;
            };
		}

	    return ThrowParameterBuilder::build_from_dynamic_reconf(sp_engine_parameter_
		                                                       ,type
		                                                       ,request);
	}

    std::string ThrowEngine::get_throw_points_as_string() const{
        return sp_current_throw_->get_debug_string();
    }

    int8_t ThrowEngine::get_movement_stage() const{
	    int8_t stage = 0;

	    while(movement_stages_.size() > stage && time_ >= movement_stages_.at(stage)){
	        ++stage;
	    }

        return stage;
    }
}