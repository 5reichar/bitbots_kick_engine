#ifndef THROW_PARAMETER_BUILDER_H
#define THROW_PARAMETER_BUILDER_H

#include <memory>
#include "parameter/throw_parameter.h"
#include "parameter/throw_engine_parameter.h"

class ThrowParameterBuilder
{
public:
    static std::shared_ptr<ThrowParameter> build_from_dynamic_reconf(std::shared_ptr<ThrowEngineParameter> & engine_parameter)
    {
        std::shared_ptr<ThrowParameter> sp_parameter;

        sp_parameter->throw_start_position_ = engine_parameter->throw_start_position_;
        sp_parameter->throw_release_position_ = engine_parameter->throw_release_position_;
        sp_parameter->pick_up_duration_share_ = engine_parameter->pick_up_duration_share_;
        sp_parameter->throw_preparation_duration_share_ = engine_parameter->throw_preparation_duration_share_;
        sp_parameter->throw_duration_share_ = engine_parameter->throw_duration_share_;
    	sp_parameter->movement_cycle_frequence_ = engine_parameter->frequency_;

        return sp_parameter;
    };

protected:

};

#endif