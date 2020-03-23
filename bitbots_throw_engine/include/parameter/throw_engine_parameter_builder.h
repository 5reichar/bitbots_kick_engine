#ifndef THROW_ENGINE_PARAMETER_BUILDER_H
#define THROW_ENGINE_PARAMETER_BUILDER_H

#include <memory>
#include "parameter/throw_engine_parameter.h"
#include <bitbots_throw_engine/throw_engine_paramsConfig.h>

class ThrowEngineParameterBuilder
{
public:
    static  std::shared_ptr<ThrowEngineParameter> build_from_dynamic_reconf(bitbots_throw_engine::throw_engine_paramsConfig& config, uint32_t level)
    {
        std::shared_ptr<ThrowEngineParameter> sp_parameter;

        sp_parameter->frequency_ = config.frequency;
        sp_parameter->hand_distance_ = config.hand_distance;
        sp_parameter->max_throw_distance_ = config.max_throw_distance;

        sp_parameter->throw_start_position_ = struct3d{config.throw_starting_position_x, config.throw_starting_position_y, config.throw_starting_position_z};
        sp_parameter->throw_release_position_ = struct3d{config.throw_release_position_x, config.throw_release_position_y, config.throw_release_position_z};

        sp_parameter->pick_up_duration_share_ = config.pick_up_ball_duration_share;
        sp_parameter->throw_preparation_duration_share_ = config.throw_preparation_duration_share;
        sp_parameter->throw_duration_share_ = config.throw_duration_share;

        return sp_parameter;
    };

protected:

};

#endif