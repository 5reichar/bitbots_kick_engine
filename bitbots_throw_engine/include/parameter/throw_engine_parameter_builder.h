#ifndef BITBOTS_THROW_THROW_ENGINE_PARAMETER_BUILDER_H
#define BITBOTS_THROW_THROW_ENGINE_PARAMETER_BUILDER_H

#include <memory>
#include "parameter/throw_engine_parameter.h"
#include <bitbots_throw/throw_engine_paramsConfig.h>

namespace bitbots_throw{
    class ThrowEngineParameterBuilder{
    public:
        static  std::shared_ptr<ThrowEngineParameter> build_from_dynamic_reconf(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level)    {
            auto sp_parameter = build_default();

            sp_parameter->frequency_ = config.frequency;
            sp_parameter->hand_distance_ = config.hand_distance;
            sp_parameter->max_throw_distance_ = config.max_throw_distance;

            sp_parameter->throw_start_position_ = Struct3d{config.throw_starting_position_x, config.throw_starting_position_y, config.throw_starting_position_z};
            sp_parameter->throw_release_position_ = Struct3d{config.throw_release_position_x, config.throw_release_position_y, config.throw_release_position_z};

            sp_parameter->pick_up_duration_share_ = config.pick_up_ball_duration_share;
            sp_parameter->throw_preparation_duration_share_ = config.throw_preparation_duration_share;
            sp_parameter->throw_duration_share_ = config.throw_duration_share;
            sp_parameter->throw_conclusion_duration_share_ = config.throw_conclusion_duration_share;
            sp_parameter->throw_anlge_ = config.throw_angle;

            return sp_parameter;
        };

        static  std::shared_ptr<ThrowEngineParameter> build_default()    {
            // TODO: enter better default values
            return std::make_shared<ThrowEngineParameter>(0.0, 0.0, 0.0, 0.0, 0.0, Struct3d{0.0, 0.0, 0.0}, Struct3d{0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0, 0.0);
        };

    protected:
    };
}
#endif //BITBOTS_THROW_THROW_ENGINE_PARAMETER_BUILDER_H