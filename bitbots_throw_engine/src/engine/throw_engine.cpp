#include "engine/throw_engine.h"

#include <math.h>

#include <utility>
#include "parameter/throw_type_parameter_builder.h"

namespace bitbots_throw{
	ThrowEngine::ThrowEngine()
					:sp_throw_factory_(new ThrowFactory()){
		reset();
	}

	ThrowResponse ThrowEngine::update(double dt){
		ThrowResponse response;

		response.ik_mode_ = sp_current_throw_->get_ik_mode(time_);
		response.support_foot_to_left_hand_ = sp_current_throw_->get_left_hand_transform(time_);
		response.support_foot_to_right_hand_ = sp_current_throw_->get_right_hand_transform(time_);
		response.support_foot_to_left_foot_ = sp_current_throw_->get_left_foot_transform(time_);
		response.support_foot_to_right_foot_ = sp_current_throw_->get_right_foot_transform(time_);

		time_ += dt;
		return response;
	}

	void ThrowEngine::reset(){
		time_ = 0.0;
		sp_current_throw_ = nullptr;
	}

	int ThrowEngine::get_percent_done() const{
		// Wrapper to keep style consistence
		return getPercentDone();
	}

	int ThrowEngine::getPercentDone() const{
	    auto duration = sp_current_throw_->get_duration();
	    return 0.0 == duration ? 100 : (int) std::round(100.0 * (time_ / duration));
	}

    void ThrowEngine::visualize_curves(std::shared_ptr<ThrowVisualizer> & sp_visualizer){
        sp_current_throw_->visualize_curves(sp_visualizer);
    }

	void ThrowEngine::set_goals(const ThrowRequest & request){
		// Wrapper to keep style consistence
		setGoals(request);
	}

	void ThrowEngine::setGoals(const ThrowRequest & request){
		auto throw_type_id = sp_throw_factory_->select_throw_type(request.goal_position_);
		auto current_movement = sp_throw_factory_->select_movement(throw_type_id, request);
		sp_current_throw_ = current_movement->create_material();
	}

	void ThrowEngine::set_throw_types(std::map<ThrowTypeId, std::shared_ptr<ThrowType>> & types){
	    sp_throw_factory_->set_throw_type_parameter(types);
	}

	void ThrowEngine::set_engine_parameter(std::shared_ptr<RobotAndWorldParameter> & parameter){
        sp_throw_factory_->set_engine_parameter(parameter);
	}

    std::string ThrowEngine::get_throw_points_as_string() const{
        return sp_current_throw_->get_debug_string();
    }

    int8_t ThrowEngine::get_movement_stage() const{
	    int8_t stage = 0;
	    auto v_stages = sp_current_throw_->get_movement_stage();

	    while(v_stages.size() > stage && time_ >= v_stages.at(stage)){
	        ++stage;
	    }

        return stage;
    }
}