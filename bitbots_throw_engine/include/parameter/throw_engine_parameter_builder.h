#ifndef BITBOTS_THROW_THROW_ENGINE_PARAMETER_BUILDER_H
#define BITBOTS_THROW_THROW_ENGINE_PARAMETER_BUILDER_H

#include <memory>
#include "parameter/throw_engine_parameter.h"
#include "parameter/throw_type_parameter_builder.h"
#include <bitbots_throw/throw_engine_paramsConfig.h>

namespace bitbots_throw{
    class ThrowEngineParameterBuilder{
    public:
        static std::shared_ptr<ThrowEngineParameter> build_from_dynamic_reconf(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level){
            auto sp_parameter = build_default();

            sp_parameter->gravity_ = config.gravity;
            sp_parameter->robot_height_ = config.robot_height;
            sp_parameter->head_height_ = config.head_height;
            sp_parameter->arm_length_ = config.arm_length;
            sp_parameter->arm_max_stall_torque_ = config.arm_max_stall_torque;
            sp_parameter->arm_stall_torque_usage_ = config.arm_stall_torque_usage;
            sp_parameter->ball_radius_ = config.ball_radius;
            sp_parameter->ball_weight_ = config.ball_weight;

            sp_parameter->default_throw_->active_ = true;
            sp_parameter->default_throw_->throw_strength_ = config.throw_strength;
            sp_parameter->default_throw_->throw_angle_ = config.throw_angle;
            sp_parameter->default_throw_->movement_duration_ = config.movement_duration;
            sp_parameter->default_throw_->movement_share_pick_up_ = config.movement_share_pick_up;
            sp_parameter->default_throw_->movement_share_preparation_ = config.movement_share_preparation;
            sp_parameter->default_throw_->movement_share_throw_ = config.movement_share_throw;
            sp_parameter->default_throw_->movement_share_conclusion_ = config.movement_share_conclusion;

            return sp_parameter;
        };

        static  std::shared_ptr<ThrowEngineParameter> build_default(){
            // TODO: enter better default values
            return std::make_shared<ThrowEngineParameter>(0.0
                                                         ,0.0
                                                         ,0.0
                                                         ,0.0
                                                         ,0.0
                                                         ,0.0
                                                         ,0.0
                                                         ,0.0
                                                         ,ThrowTypeParameterBuilder::build_empty_throw_type());
        };

    protected:
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_ENGINE_PARAMETER_BUILDER_H