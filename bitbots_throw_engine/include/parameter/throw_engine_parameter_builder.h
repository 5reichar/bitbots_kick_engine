#ifndef THROW_ENGINE_PARAMETER_BUILDER_H
#define THROW_ENGINE_PARAMETER_BUILDER_H

#include "parameter/throw_engine_parameter.h"
#include <bitbots_throw_engine/throw_engine_paramsConfig.h>

class ThrowEngineParameterBuilder
{
public:
    static ThrowEngineParameter build_from_dynamic_reconf(bitbots_throw_engine::throw_engine_paramsConfig& config, uint32_t level)
    {
        std::shared_ptr<ThrowEngineParameter> sp_parameter;

        return sp_parameter;
    };

protected:

};

#endif