#ifndef BITBOTS_THROW_THROW_NODE_PARAMETER_H
#define BITBOTS_THROW_THROW_NODE_PARAMETER_H

namespace bitbots_throw{
    struct ThrowNodeParameter{
        bool debug_active_;
        // Max frequency of engine update rate [hz]
        double engine_frequency_;
        // Publish odom every [int] update of the engine
        int odom_publish_factor_;
        // Timeout time for bioIK [s]
        double bio_ik_time_;
        // Show if the code is run in a simulation or on the robot
        bool simulation_active_;

        //////		Constructor
        ThrowNodeParameter(bool debug_active
                          ,double engine_frequency
                          ,int odom_publish_factor
                          ,double bio_ik_time
                          ,bool simulation_active
                          )
                          :debug_active_{debug_active}
                          ,engine_frequency_{engine_frequency}
                          ,odom_publish_factor_{odom_publish_factor}
                          ,bio_ik_time_{bio_ik_time}
                          ,simulation_active_{simulation_active}{
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_NODE_PARAMETER_H