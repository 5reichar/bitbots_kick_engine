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
        // activate the visualization of the limbs points as lines
        bool visualize_limbs_;
        // activate the visualization of the left arm points as arrows
        bool visualize_left_arm_;
        // activate the visualization of the right arm points as arrows
        bool visualize_right_arm_;
        // activate the visualization of the left foot points as arrows
        bool visualize_left_foot_;
        // activate the visualization of the right foot points as arrows
        bool visualize_right_foot_;
        // Use Gradient colors for the visualization of the arrows
        bool visualize_arrows_use_gradient_;

        //////		Constructor
        ThrowNodeParameter(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level){
            debug_active_ = config.debug_active;
            engine_frequency_ = config.engine_frequency;
            odom_publish_factor_ = config.odom_pub_factor;
            bio_ik_time_ = config.bio_ik_time;
            visualization_smoothness_ = config.visualization_smoothness;
            visualize_limbs_ = config.visualize_limbs;
            visualize_left_arm_ = config.visualize_left_arm;
            visualize_right_arm_ = config.visualize_right_arm;
            visualize_left_foot_ = config.visualize_left_foot;
            visualize_right_foot_ = config.visualize_right_foot;
            visualize_arrows_use_gradient_ = config.visualize_arrows_use_gradient;
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_NODE_PARAMETER_H