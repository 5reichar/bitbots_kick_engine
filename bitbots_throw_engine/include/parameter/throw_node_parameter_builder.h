#ifndef THROW_NODE_PARAMETER_BUILDER_H
#define THROW_NODE_PARAMETER_BUILDER_H

#include <memory>
#include "parameter/throw_node_parameter.h"
#include <bitbots_throw/throw_engine_paramsConfig.h>

namespace bitbots_throw{
    class ThrowNodeParameterBuilder{
    public:
        static std::shared_ptr<ThrowNodeParameter> build_from_dynamic_reconf(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level){
            auto sp_parameter = build_default();

            sp_parameter->debug_active_ = config.debug_active;
            sp_parameter->engine_frequency_ = config.engine_frequency;
            sp_parameter->odom_publish_factor_ = config.odom_pub_factor;
            sp_parameter->bio_ik_time_ = config.bio_ik_time;

            return sp_parameter;
        };

        static std::shared_ptr<ThrowNodeParameter> build_default(){
            // TODO: enter better default values
            return std::make_shared<ThrowNodeParameter>(false, 0.0, 0, 0.0, false);
        };

    protected:
    };
} //bitbots_throw
#endif //THROW_NODE_PARAMETER_BUILDER_H