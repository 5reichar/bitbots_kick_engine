#ifndef BITBOTS_THROW_THROW_NODE_PARAMETER_H
#define BITBOTS_THROW_THROW_NODE_PARAMETER_H

#include <bitbots_throw/throw_engine_paramsConfig.h>

namespace bitbots_throw{
    struct ThrowNodeParameter{
        bool debug_active_;
        // Max frequency of engine update rate [hz]
        double engine_frequency_;
        // Publish odom every [int] update of the engine
        int odom_publish_factor_;
        // Timeout time for bioIK [s]
        double bio_ik_time_;
        // Frequency of the point update rate for the visualization [in hz, > 0]
        double visualization_smoothness_;
        // Show if the code is run in a simulation or on the robot
        bool simulation_active_;

        //////		Constructor
        ThrowNodeParameter(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level){
            debug_active_ = config.debug_active;
            engine_frequency_ = config.engine_frequency;
            odom_publish_factor_ = config.odom_pub_factor;
            bio_ik_time_ = config.bio_ik_time;
            visualization_smoothness_ = config.visualization_smoothness;
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_NODE_PARAMETER_H