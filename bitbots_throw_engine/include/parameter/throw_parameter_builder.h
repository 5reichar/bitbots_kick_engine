#ifndef THROW_PARAMETER_BUILDER_H
#define THROW_PARAMETER_BUILDER_H

#include <memory.h>
#include "parameter/throw_parameter.h"
#include <bitbots_throw_engine/throw_engine_paramsConfig.h>

class ThrowParameterBuilder
{
public:
    static ThrowParameter build_from_dynamic_reconf(bitbots_throw_engine::throw_engine_paramsConfig& config, uint32_t level)
    {
        std::shared_ptr<ThrowParameter> sp_parameter;

        return sp_parameter;
    };

protected:

};

#endif