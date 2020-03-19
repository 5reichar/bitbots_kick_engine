#ifndef THROW_NODE_PARAMETER_BUILDER_H
#define THROW_NODE_PARAMETER_BUILDER_H

#include "parameter/throw_node_parameter.h"
#include <bitbots_throw_engine/throw_engine_paramsConfig.h>

class ThrowNodeParameterBuilder
{
public:
    static ThrowNodeParameter build_from_dynamic_reconf(bitbots_throw_engine::throw_engine_paramsConfig& config, uint32_t level)
    {
        std::shared_ptr<ThrowNodeParameter> sp_parameter;

        return sp_parameter;
    };

protected:

};

#endif